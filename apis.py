import socket
import time
import sys
from NatNetClient import NatNetClient
from util import quaternion_to_euler_angle_vectorized1
import numpy as np
import cv2
from enum import Enum


positions = {}
rotations = {}

# This is a callback function that gets connected to the NatNet client. It is called once per rigid body per frame
def receive_rigid_body_frame(robot_id, position, rotation_quaternion):
    # Position and rotation received
    positions[robot_id] = position
    # The rotation is in quaternion. We need to convert it to euler angles

    rotx, roty, rotz = quaternion_to_euler_angle_vectorized1(rotation_quaternion)

    rotations[robot_id] = rotz

class Position:

    def __init__(self, clientAddress, optitrackServerAddress, robot_id):
        self.robot_id = robot_id
        # This will create a new NatNet client
        self.streaming_client = NatNetClient()
        self.streaming_client.set_client_address(clientAddress)
        self.streaming_client.set_server_address(optitrackServerAddress)
        self.streaming_client.set_use_multicast(True)
        # Configure the streaming client to call our rigid body handler on the emulator to send data out.
        self.streaming_client.rigid_body_listener = receive_rigid_body_frame
        if not self.streaming_client.run():
            raise Exception("Not running")

    def get(self):
        xyz, rot = self.get_optitrack()
        return (np.array([xyz[0], xyz[1]]), rot / 180.0 * np.pi)

    def get_optitrack(self):
        while True:
            if self.robot_id in positions:
                #print('Last position', positions[robot_id], ' rotation', rotations[robot_id])
                return (positions[self.robot_id], rotations[self.robot_id])

class Robot:

    def set_motor(self, a, b, c, d):
        pass

    def stop_motor(self):
        pass

    def shutdown(self):
        pass

class RemoteRobot(Robot):

    def __init__(self, IP_ADDRESS):
        super().__init__()
        # Connect to the robot
        try:
            self.s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.s.connect((IP_ADDRESS, 5000))
            print('Connected to robot')
        except Exception:
            raise Exception("Could not connect to robot")

    def set_motor(self, a, b, c, d):
        command = 'CMD_MOTOR#%d#%d#%d#%d\n'%(int(a), int(b), int(c), int(d))
        self.s.send(command.encode('utf-8'))

    def stop_motor(self):
        command = 'CMD_MOTOR#00#00#00#00\n'
        self.s.send(command.encode('utf-8'))

    def shutdown(self):
        self.s.shutdown(2)
        self.s.close()

class LocalRobot(Robot):

    def __init__(self):
        super().__init__()
        # Connect to the robot
        import Motor
        self.motor = Motor.Motor()

    def set_motor(self, a, b, c, d):
        self.motor.setMotorModel(int(a), int(b), int(c), int(d))

    def stop_motor(self):
        self.set_motor(0, 0, 0, 0)

    def shutdown(self):
        self.stop_motor()

def angle_diff(desired, actual):
    return np.arctan2(np.sin(desired - actual) , np.cos(desired - actual));

def dist(x_d, x_t):
    err = x_d - x_t
    dist = np.linalg.norm(err)
    return dist

def angle(x_d, x_t):
    err = x_d - x_t
    return np.arctan2(err[1], err[0])


def dist_and_angle(x_d, x_t):
    err = x_d - x_t
    dist = np.linalg.norm(err)
    angle = np.arctan2(err[1], err[0])
    return (dist, angle)

class Duck(Enum):
    SMALL_DUCK = ([0, 78, 185],[32, 255, 255])

class Camera:

    def __init__(self, raspberrypi = False, read_img = False, img_name = ""):
        
        self.rpi = raspberrypi
      
        self.read_img = read_img
        self.img_name = img_name

        if not read_img:

            if raspberrypi:
                from picamera2 import Picamera2 
                self.camera = Picamera2()
                self.camera.configure(self.camera.create_preview_configuration(main={"format": 'XRGB8888', "size": (640, 480)}))
                self.camera.start()
            else:
                self.camera = cv2.VideoCapture(0)
    
    def close(self):
        if not self.rpi and not self.read_img:
            self.camera.release()

    def get(self):
        if self.read_img:
            img = cv2.imread(self.img_name)
            return cv2.resize(img, (640, 480))
        if self.rpi:
            return self.camera.capture_array()
        else:
            _, frame = self.camera.read()
            return frame

    def get_blobs_and_gray(self, color = Duck.SMALL_DUCK):
        frame = self.get()
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        # 95 to 125 for HUE
        # saturation in 25 to 255

        # orange 15 to 25
        # saturation 25 to 255

        lower_val = np.array(color.value[0])
        upper_val = np.array(color.value[1])
        mask = cv2.inRange(hsv, lower_val, upper_val)
        #
        result = cv2.bitwise_and(frame, frame, mask = mask)

        
        kernel = np.ones((11,11),np.uint8)

        #result = cv2.morphologyEx(result, cv2.MORPH_OPEN, kernel)
        #result = cv2.morphologyEx(result, cv2.MORPH_CLOSE, kernel)
        #
        #cv2.imshow('result', result)

        result = cv2.bitwise_not(result)
        
        gray = cv2.cvtColor(result, cv2.COLOR_BGR2GRAY)

        # Display the resulting frame
        gray = cv2.medianBlur(gray, 11)

        #_, gray = cv2.threshold(gray, 100, 255, cv2.THRESH_BINARY)

        params = cv2.SimpleBlobDetector_Params()

        params.minThreshold = 100;
        params.maxThreshold = 255;
         
        # Filter by Area.
        params.filterByArea = True
        params.minArea = 1000
        params.maxArea = 2**64
         
        # Filter by Circularity
        params.filterByCircularity = False
        params.minCircularity = 0.1
         
        # Filter by Convexity
        params.filterByConvexity = False
        params.minConvexity = 0.87
         
        # Filter by Inertia
        params.filterByInertia = False
        params.minInertiaRatio = 0.01

        detector = None

        ver = (cv2.__version__).split('.')
        if int(ver[0]) < 3 :
          detector = cv2.SimpleBlobDetector(params)
        else : 
          detector = cv2.SimpleBlobDetector_create(params)
      
        keypoints = detector.detect(gray)

        im_with_keypoints = cv2.drawKeypoints(gray, keypoints, np.array([]), (0,0,255), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)


        blobs = []

        for k in keypoints:
            blobs.append([k.pt[0], k.pt[1], k.size])

        if len(blobs) == 0:
            return None, im_with_keypoints

        return np.array([blobs]), im_with_keypoints
    
    def get_blobs(self, color = Duck.SMALL_DUCK):
        blobs, _ = self.get_blobs_and_gray(color)
        return blobs
    
    def get_largest_blob(self, color = Duck.SMALL_DUCK):
        blobs = self.get_blobs(color)
        
        if blobs is not None:
            blobs = np.uint16(np.around(blobs))
            
            max_radius = -1
            max_circle = None

            for i in blobs[0, :]:
                radius = i[2]
                if radius > max_radius:
                    max_radius = radius
                    max_circle = i
            return max_circle
        else:
            return None
 
    def get_largest_blob_and_img(self, color = Duck.SMALL_DUCK):
        blobs, gray = self.get_blobs_and_gray(color)
        
        if blobs is not None:
            blobs = np.uint16(np.around(blobs))
            
            max_radius = -1
            max_circle = None

            for i in blobs[0, :]:
                radius = i[2]
                if radius > max_radius:
                    max_radius = radius
                    max_circle = i
            return (max_circle, gray)
        else:
            return (None, gray)
    
