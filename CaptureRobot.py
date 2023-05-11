from apis import *
import time

class CaptureRobot:
    """ This robot captures a duck """
    
    def __init__(self, robot, camera):

        self.robot = robot
        self.camera = camera

        self.ram_time = 1.0
        self.ramming = False
        self.start = time.time()

        
    def step(self) -> bool:
        """ Returns true when done """

        blob = self.camera.get_largest_blob()
        
        w = 0.0
        v = 0.0

        if blob is not None and not self.ramming:
            self.start = time.time()
            
            if abs((320.0 - blob[0])) < 20.0:
                print("Ramming it", 320.0 - blob[0])
                w = 0.0
                v = 1500.0
                self.ramming = True
            else:
                sign = 1.0
                if 320.0 - blob[0] < 0.0:
                    sign = -1.0
                w = sign * 800.0
                print("Rotating", w)

        elif self.ramming and time.time() - self.start < self.ram_time:
            w = 0.0
            v = 1500.0

        elif time.time() - self.start >= self.ram_time:
            robot.set_motor(0, 0, 0, 0)
            return True
        
        u = np.array([v - w, v + w])

        robot.set_motor(u[0], u[0], u[1], u[1])
        return False

