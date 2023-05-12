from apis import *
import time
from enum import Enum

class CaptureSolution(Enum):
    FINISHED = 1
    FAILED = 2
    WORKING = 3

def theta_duck(u):
    return -0.00172753493 * u + 0.5294127344

class CaptureRobot:
    """ This robot captures a duck """
    
    def __init__(self, robot, position, camera):

        self.robot = robot
        self.position = position
        self.camera = camera

        self.ram_time = 1.0
        self.ramming = False
        self.start = time.time()
        self.have_angle = False
        self.angle = 0.0
        self.K2 = 1500.0
        self.timeout = 5.0
        self.timeout_start = time.time()

        
    def step(self) -> CaptureSolution:
        """ Returns true when done """

        if not self.ramming and time.time() - self.timeout_start > self.timeout:
            return CaptureSolution.FAILED

        w = 0.0
        v = 0.0

        if not self.have_angle:
            print("Getting angle")
            self.robot.set_motor(0, 0, 0, 0)
            print("Getting blob")
            blob = self.camera.get_largest_blob()
            print("Got blob")
            if blob is not None:
                _, opt_angle = self.position.get()
                print(theta_duck(blob[0]), opt_angle)
                self.angle = np.arctan2(np.sin(theta_duck(blob[0]) + opt_angle), np.cos(theta_duck(blob[0]) + opt_angle))
                print(self.angle)
                self.have_angle = True
            else:
                print("Failed")
                return CaptureSolution.FAILED
        
        if self.have_angle and not self.ramming:
            self.start = time.time()
            _, opt_angle = self.position.get()
            
            print("Angle diff is", angle_diff(self.angle, opt_angle), self.angle, opt_angle)
            if abs(angle_diff(self.angle, opt_angle)) < 0.02:
                #print("Ramming it", 320.0 - blob[0])
                w = 0.0
                v = 1500.0
                self.ramming = True
            else:
                sign = 1
                if angle_diff(self.angle, opt_angle) < 0.0:
                    sign = -1
                w = sign * 900.0 #self.K2 * angle_diff(self.angle, angle)
                v = 0.0
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

    capture_robot = CaptureRobot(robot, position, camera)

    try:
        while capture_robot.step() == CaptureSolution.WORKING:
            time.sleep(0.1)

    except KeyboardInterrupt as e:
        print("Sending stop")
        
    robot.set_motor(0.0, 0.0, 0.0, 0.0)
    robot.stop_motor()
    robot.shutdown()
    cv2.destroyAllWindows()

    os._exit(0)