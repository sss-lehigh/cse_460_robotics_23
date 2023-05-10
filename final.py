from apis import *
import time
from enum import Enum
from ExploringRobot import *

class State(Enum):
    EXPLORING = 1
    CAPTURING = 2
    RETURNING_TO_SAFETY = 3
    RETURNING_TO_EXPLORE = 4

if __name__ == "__main__":
   
    robot = RemoteRobot("192.168.0.207") 
    position = Position("192.168.0.153", "192.168.0.172", 307)
    grid = Discretization(-3.3, -.26, -.46, 2.63)

    print("Starting")

    robot.stop_motor()

    # 1250, 1500
    exploring = ExploringRobot(robot, position, grid, 1250, 1500)

    state = State.EXPLORING

    #led = RobotLight()

    #led.blue()

    try:
        while True:

            if state == State.EXPLORING: 
                exploring.step()
            elif state == State.CAPTURING:
                pass
            elif state == State.RETURNING_TO_SAFETY:
                pass
            elif state == State.RETURNING_TO_EXPLORE:
                pass
            else:
                print("Error")

            time.sleep(0.1)

    except KeyboardInterrupt as e:
        print("Sending stop")
        robot.set_motor(0.0, 0.0, 0.0, 0.0)
        robot.stop_motor()
        robot.shutdown()

    os._exit(0)
