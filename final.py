from apis import *
import time
from enum import Enum
from ExploringRobot import *

class State(Enum):
    EXPLORING = 1
    CAPTURING = 2
    RETURNING_TO_SAFETY = 3

if __name__ == "__main__":
   
    robot = RemoteRobot("192.168.0.207") 
    position = Position("192.168.0.153", "192.168.0.172", 307)
    camera = Camera(remote = True, port = 8080, ip_addr = "192.168.0.207")

    grid = Discretization(-3.3, -.26, -.46, 2.63)
    home_node = 0

    print("Starting")

    robot.stop_motor()

    # 1250, 1500
    exploring = ExploringRobot(robot, position, grid, 1250, 1500)
    capture = None
    go_home = None

    state = State.EXPLORING

     

    #led = RobotLight()

    #led.blue()

    try:
        while True:

            if state == State.EXPLORING: 
                if camera.get_largest_blob() != None:
                    print("Switching to capture")
                    state = State.CAPTURING
                    capture = CaptureRobot(robot, camera)
                    exploring = None
                    continue
                exploring.step()
            elif state == State.CAPTURING:
                if capture.step():
                    print("Switching to going home")
                    state = State.RETURNING_TO_SAFETY
                    capture = None
                    go_home = GoHomeRobot(robot, position, grid, 1250, 1500, home_node)
            elif state == State.RETURNING_TO_SAFETY:
                if go_home.step():
                    print("Switching to explore")
                    state = State.EXPLORING
                    go_home = None
                    exploring = ExploringRobot(robot, position, grid, 1250, 1500)
            else:
                print("Error")

            time.sleep(0.1)

    except KeyboardInterrupt as e:
        print("Sending stop")
        robot.set_motor(0.0, 0.0, 0.0, 0.0)
        robot.stop_motor()
        robot.shutdown()

    os._exit(0)
