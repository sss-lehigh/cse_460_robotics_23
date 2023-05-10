from apis import *
import time

class ExploringRobot:
    
    def __init__(self, robot, position, grid, K1, K2):
        self.robot = robot
        self.position = position
        self.grid = grid
        self.K1 = K1
        self.K2 = K2
        self.start = time.time()
        self.factor = 2.0
        self.node_idx = 1

        x_t, _ = position.get()

        curr_node = self.grid.get_nearest_node(x_t)

        print("Starting at", curr_node)

        self.get_to = 0

        self.path = self.grid.get_path(curr_node, self.get_to)
        
        self.init = np.zeros(2)
        self.slope = np.zeros(2)

        while self.node_idx >= len(self.path):
            self.get_to += 1
            if self.get_to >= self.grid.num_nodes():
                break
            self.path = self.grid.get_path(curr_node, self.get_to)
            print("Path from", curr_node, "to", self.get_to)
        
        if self.node_idx < len(self.path):
            self.init = x_t
            self.slope = self.path[self.node_idx] - self.init
        else:
            self.slope = np.zeros(2)
            self.init = self.path[-1]

    def step(self):
        x_t, angle = self.position.get()

        curr_time = time.time()
        curr_node = self.grid.get_nearest_node(x_t)

        if (curr_time - self.start) / self.factor > 1.0:
            self.start = curr_time
            self.node_idx += 1
            if self.node_idx < len(self.path):
                self.init = x_t
                self.slope = self.path[self.node_idx] - self.init
            else:
                print("Updating path")
                self.node_idx = 1
                self.get_to += 1
                if self.get_to >= self.grid.num_nodes():
                    self.get_to = 0
                print("Path from", curr_node, "to", self.get_to)
                self.path = self.grid.get_path(curr_node, self.get_to)

                while self.node_idx >= len(self.path):
                    self.get_to += 1
                    if self.get_to >= self.grid.num_nodes():
                        self.get_to = 0
                    print("Path from", curr_node, "to", self.get_to)
                    self.path = self.grid.get_path(curr_node, self.get_to)
                if self.node_idx < len(self.path):
                    self.init = x_t
                    self.slope = self.path[self.node_idx] - self.init
                else:
                    self.slope = np.zeros(2)
                    self.init = self.path[-1]

        x_d = ((curr_time - self.start) / self.factor) * self.slope + self.init

        dist, desired = dist_and_angle(x_d, x_t)

        delta_angle = angle_diff(desired, angle)

        v = self.K1 * dist
        w = self.K2 * delta_angle
        u = np.array([v - w, v + w])

        self.robot.set_motor(u[0], u[0], u[1], u[1])

