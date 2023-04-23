import socket

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