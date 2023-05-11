from apis import *
import time

class DropOffRobot:
    """ This robot drops off a duck """
    
    def __init__(self, robot):

        self.robot = robot

        self.drop_off_time = 1.0
        self.start = time.time()

        
    def step(self) -> bool:
        """ Returns true when done """

        if time.time() - self.start >= self.drop_off_time:
            self.robot.set_motor(0, 0, 0, 0)
            return True
        else: 
            self.robot.set_motor(-1500, -1500, -1500, -1500)
        return False

