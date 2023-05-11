from apis import *
import time
from enum import Enum

class CaptureSolution(Enum):
    FINISHED = 1
    FAILED = 2
    WORKING = 3

class CaptureRobot:
    """ This robot captures a duck """
    
    def __init__(self, robot, camera):

        self.robot = robot
        self.camera = camera

        self.ram_time = 1.0
        self.ramming = False
        self.start = time.time()

        
    def step(self) -> CaptureSolution:
        """ Returns true when done """

        blob = self.camera.get_largest_blob()
        
        w = 0.0
        v = 0.0

        if blob is not None and not self.ramming:
            self.start = time.time()
            
            if abs((320.0 - blob[0])) < 20.0:
                #print("Ramming it", 320.0 - blob[0])
                w = 0.0
                v = 1500.0
                self.ramming = True
            else:
                sign = 1.0
                if 320.0 - blob[0] < 0.0:
                    sign = -1.0
                w = min(3.0 * (320.0 - blob[0]), 800.0 * sign)
                print("Rotating", w)

        elif self.ramming and time.time() - self.start < self.ram_time:
            w = 0.0
            v = 1500.0

        elif time.time() - self.start >= self.ram_time:
            self.robot.set_motor(0, 0, 0, 0)
            if self.ramming:
                return CaptureSolution.FINISHED
            return CaptureSolution.FAILED
        
        u = np.array([v - w, v + w])

        self.robot.set_motor(u[0], u[0], u[1], u[1])
        return CaptureSolution.WORKING

if __name__ == "__main__":
   
    robot = RemoteRobot("192.168.0.207") 
    position = Position("192.168.0.164", "192.168.0.172", 307)
    camera = Camera(remote = True, port = 8080, ip_addr = "192.168.0.207")

    capture_robot = CaptureRobot(robot, camera)

    try:
        while capture_robot.step() == CaptureSolution.WORKING:
            time.sleep(0.2)

    except KeyboardInterrupt as e:
        print("Sending stop")
        
    robot.set_motor(0.0, 0.0, 0.0, 0.0)
    robot.stop_motor()
    robot.shutdown()

    os._exit(0)