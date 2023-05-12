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
    f.close()

    robot = RemoteRobot(config["robot_ip"]) 
    position = Position(config["local_ip"], "192.168.0.172", config["robot_id"])
    camera = Camera(remote = True, port = 8080, ip_addr = config["robot_ip"])
    
    heartbeat_server = DistributionServer(8090)
    other_computer = config["other_computer_ip"]
    heartbeat_manager = Distribution(heartbeat_server, {0 : "http://" + other_computer + ":8090"})

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

    grid = Discretization(-3.3, -.26, -.46, 2.63)
    home_node = 0

    print("Starting")

    robot.stop_motor()

    # 1250, 1500
    exploring = ExploringRobot(robot, position, grid, 1250, 1500, 0)
    capture = None
    go_home = None
    drop_off = None

    state = State.EXPLORING

     

    #led = RobotLight()

    #led.blue()

    try:
        while True:

            if state == State.EXPLORING: 
                if camera.get_largest_blob() is not None:
                    robot.stop_motor()
                    print("Switching to capture")
                    state = State.CAPTURING
                    capture = CaptureRobot(robot, position, camera)
                    exploring = None
                    continue
                exploring.step()
            elif state == State.CAPTURING:
                res = capture.step()
                if res is CaptureSolution.FINISHED:
                    print("Switching to going home")
                    state = State.RETURNING_TO_SAFETY
                    capture = None
                    go_home = GoHomeRobot(robot, position, grid, 1250, 1500, home_node)
                elif res is CaptureSolution.FAILED:
                    print("Switching to explore")
                    state = State.EXPLORING
                    capture = None
                    x_t, _ = position.get()
                    exploring = ExploringRobot(robot, position, grid, 1250, 1500, grid.get_nearest_node(x_t))
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
                    exploring = ExploringRobot(robot, position, grid, 1250, 1500, grid.get_nearest_node(x_t))
            else:
                print("Error")

            time.sleep(0.1)

    except KeyboardInterrupt as e:
        print("Sending stop")
        robot.set_motor(0.0, 0.0, 0.0, 0.0)
        robot.stop_motor()
        robot.shutdown()

    os._exit(0)
