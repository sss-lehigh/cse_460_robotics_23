from apis import *
import time
from enum import Enum
from ExploringRobot import *
from CaptureRobot import *
from GoHomeRobot import *
from DropOffRobot import *
from Distribution import *
import json

other_died = False

class State(Enum):
    EXPLORING = 1
    CAPTURING = 2
    RETURNING_TO_SAFETY = 3
    DROP_OFF = 4

if __name__ == "__main__":
    f = open("./config.json")
    config = json.load(f)

    print("Using config:")
    print(config)

    f.close()

    robot = RemoteRobot(config["robot_ip"]) 
    position = Position(config["local_ip"], config["opt_track"], config["robot_id"])
    camera_thread = CameraThread(Camera(remote = True, port = 8080, ip_addr = config["robot_ip"]))

    grid = Discretization(-3.9, 5.7, -2.16, 3.29) #Discretization(-3.3, -.26, -.46, 2.63)
    first_half = config["first_half"]
    home_node = 73
    if not first_half:
        home_node = 58 
    safe_nodes = [73, 58, 44, 74]
    safe_node_border = [72, 73, 58, 44, 74, 43, 57, 42, 88, 89, 87, 59]
    obstacles = [39, 24, 23, 22, 21, 36, 37, 38, 85, 100, 99, 114, 98, 97, 82, 83, 84, 69, 
                 68, 78, 79, 94, 65, 122, 106, 107, 121, 101, 116, 117, 102, 55, 40, 41, 39, 54, 97, 96, 111, 112, 32, 33]

    for x in obstacles:
        grid.remove_node(x)

    robot.stop_motor()
    
    # 1250, 1500 should start by going to node 0
    exploring = None
    capture = None
    go_home = None
    drop_off = None

    K1 = 1250
    K2 = 1500

    nodes = list()

    if config["distribute"]:

        first_half = config["first_half"]

        heartbeat_server = DistributionServer(8090)
        other_computer = config["other_computer_ip"]
        heartbeat_manager = Distribution(heartbeat_server, {0 : "http://" + other_computer + ":8090"})
        
        
        if first_half:
            for i in range(0, int(grid.num_nodes() / 2)):
                if i not in obstacles and i not in safe_nodes:
                    nodes.append(i)
        else:
            for i in range(int(grid.num_nodes() / 2), int(grid.num_nodes())):
                if i not in obstacles and i not in safe_nodes:
                    nodes.append(i)
    
        exploring = ExploringRobot(robot, position, grid, K1, K2, 0, nodes)
        while True:
            try:
                if heartbeat_manager.call(0, "heartbeat"):
                    break
            except KeyboardInterrupt:
                os._exit(0)
            except Exception:
                pass
            time.sleep(1)

        def heartbeat_target():
            global other_died
            while True:
                try:
                    heartbeat_manager.call(0, "heartbeat")
                    time.sleep(5)
                except Exception as e:
                    print("Failure", e)
                    other_died = True
                    return

        t = threading.Thread(target = heartbeat_target)
        t.start()

    else:
        for i in range(0, grid.num_nodes()):
            if i not in obstacles and i not in safe_nodes:
                nodes.append(i)
        exploring = ExploringRobot(robot, position, grid, K1, K2, 0, nodes)
    
    print("Starting")
    camera_thread.start()

    state = State.EXPLORING

    print("Here") 

    #led = RobotLight()

    #led.blue()

    curr_explore_node = 0

    try:
        while True:

            if other_died:
                print("Updating explore robot")
                nodes.clear()
                for i in range(0, grid.num_nodes()):
                    if i not in obstacles and i not in safe_nodes:
                        nodes.append(i)
                if state == State.EXPLORING:
                    exploring = ExploringRobot(robot, position, grid, K1, K2, curr_explore_node, nodes)

            if state == State.EXPLORING:
                x_t, _ = position.get()
                if grid.get_nearest_node(x_t) not in safe_node_border:
                    if camera_thread.blob is not None:
                        robot.stop_motor()
                        print("Switching to capture")
                        state = State.CAPTURING
                        capture = CaptureRobot(robot, position, camera_thread)
                        exploring = None
                        print("Was exploring", curr_explore_node)
                        continue
                curr_explore_node = exploring.step()
            elif state == State.CAPTURING:
                res = capture.step()
                if res is CaptureSolution.FINISHED:
                    print("Switching to going home")
                    state = State.RETURNING_TO_SAFETY
                    capture = None
                    go_home = GoHomeRobot(robot, position, grid, K1, K2, home_node)
                elif res is CaptureSolution.FAILED:
                    print("Switching to explore")
                    state = State.EXPLORING
                    capture = None
                    x_t, _ = position.get()
                    # exploring robot starts going to nearest node
                    exploring = ExploringRobot(robot, position, grid, K1, K2, curr_explore_node, nodes)
            elif state == State.RETURNING_TO_SAFETY:
                if go_home.step():
                    print("Switching to drop off")
                    state = State.DROP_OFF
                    go_home = None
                    drop_off = DropOffRobot(robot)
            elif state == State.DROP_OFF:
                if drop_off.step():
                    print("Switching to explore")
                    state = State.EXPLORING
                    drop_off = None
                    x_t, _ = position.get()
                    # exploring robot starts going to nearest node
                    exploring = ExploringRobot(robot, position, grid, K1, K2, curr_explore_node, nodes)
            else:
                print("Error")

            time.sleep(0.1)

    except KeyboardInterrupt as e:
        print("Sending stop")
        robot.set_motor(0.0, 0.0, 0.0, 0.0)
        robot.stop_motor()
        robot.shutdown()
        camera_thread.signal_thread()
        camera_thread.join()

    os._exit(0)
