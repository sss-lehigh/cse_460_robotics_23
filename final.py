from apis import *

if __name__ == "__main__":
    
    robot = LocalRobot()

    try:
        while True:
            robot.stop_motor()
    except (Exception, KeyboardInterrupt) as e:
        print(type(e))

    robot.shutdown()
