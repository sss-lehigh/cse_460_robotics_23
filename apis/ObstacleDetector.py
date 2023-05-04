import numpy as np

class ObstacleDetector:
    def __init__(self):
        import Ultrasonic
        self.sensor = Ultrasonic.Ultrasonic()
        self.dist = 100
    
    def sense(self):
        self.dist = self.sensor.get_distance()
        if self.dist < 25:
            print(self.dist)
            return True
        else:
            return False

    def get_loc(self, x_t, angle):
        self.dist = self.sensor.get_distance()
        dist = (self.dist + 14.0) / 100.0
        p_obstacle = np.array([np.cos(angle) * dist, np.sin(angle) * dist])
        T = np.array([[np.cos(angle), -np.sin(angle), x_t[0]],
                  [np.sin(angle), np.cos(angle), x_t[1]],
                  [0., 0., 1.]])
    
        vec = np.array([p_obstacle[0], p_obstacle[1], 1.])
    
        res = np.dot(T, vec)

        return np.array([res[0], res[1]])

    def get_node(self, x_t, angle, grid):
        loc = self.get_loc(x_t, angle)
        return grid.get_nearest_node(loc)