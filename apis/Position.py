from NatNetClient import NatNetClient
from util import quaternion_to_euler_angle_vectorized1
import numpy as np

positions = {}
rotations = {}

# This is a callback function that gets connected to the NatNet client. It is called once per rigid body per frame
def __receive_rigid_body_frame(robot_id, position, rotation_quaternion):
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
        self.streaming_client.rigid_body_listener = __receive_rigid_body_frame
        if not self.streaming_client.run():
            raise Exception("Not running")

    def get(self):
        xyz, rot = self.__get_optitrack()
        return (np.array([xyz[0], xyz[1]]), rot / 180.0 * np.pi)

    def __get_optitrack(self):
        while True:
            if self.robot_id in positions:
                #print('Last position', positions[robot_id], ' rotation', rotations[robot_id])
                return (positions[self.robot_id], rotations[self.robot_id])
