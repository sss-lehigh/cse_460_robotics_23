from apis import *
import time
from enum import Enum

class State(Enum):
    EXPLORING = 1
    CAPTURING = 2
    RETURNING_TO_SAFETY = 3
    RETURNING_TO_EXPLORE = 4

if __name__ == "__main__":
   
    robot = LocalRobot() 
    position = Position("192.168.0.207", "192.168.0.172", 307)
    grid = Discretization(-3.3, -.26, -.46, 2.63)

    print("Starting")

    x_d = np.array([0., 0.])

    x_t, angle = position.get()

    curr_node = grid.get_nearest_node(x_t)

    print("Starting at", curr_node)

    get_to = 0

    path = grid.get_path(curr_node, get_to)

    K1 = 1250
    K2 = 1500

    node_idx = 1

    start = time.time()

    init = np.copy(x_t)

    init = np.zeros(2)
    slope = np.zeros(2)

    while node_idx >= len(path):
        get_to += 1
        if get_to >= grid.num_nodes():
            break
        path = grid.get_path(curr_node, get_to)
        print("Path from", curr_node, "to", get_to)
    
    if node_idx < len(path):
        init = x_t
        slope = path[node_idx] - init
    else:
        slope = np.zeros(2)
        init = path[-1]

    factor = 2.0

    robot.stop_motor()

    state = State.EXPLORING

    try:
        while True:

            if state == State.EXPLORING: 
                x_t, angle = position.get()

                curr_time = time.time()
                curr_node = grid.get_nearest_node(x_t)

                if (curr_time - start) / factor > 1.0:
                    start = curr_time
                    node_idx += 1
                    if node_idx < len(path):
                        init = x_t
                        slope = path[node_idx] - init
                    else:
                        print("Updating path")
                        node_idx = 1
                        get_to += 1
                        if get_to >= grid.num_nodes():
                            get_to = 0
                        print("Path from", curr_node, "to", get_to)
                        path = grid.get_path(curr_node, get_to)

                        while node_idx >= len(path):
                            get_to += 1
                            if get_to >= grid.num_nodes():
                                get_to = 0
                            print("Path from", curr_node, "to", get_to)
                            path = grid.get_path(curr_node, get_to)
                        if node_idx < len(path):
                            init = x_t
                            slope = path[node_idx] - init
                        else:
                            slope = np.zeros(2)
                            init = path[-1]

                x_d = ((curr_time - start) / factor) * slope + init

                dist, desired = dist_and_angle(x_d, x_t)

                delta_angle = angle_diff(desired, angle)

                v = K1 * dist
                w = K2 * delta_angle
                u = np.array([v - w, v + w])

                robot.set_motor(u[0], u[0], u[1], u[1])
            elif state == State.CAPTURING:
                pass
            elif state == State.RETURNING_TO_SAFETY:
                pass
            elif state == State.RETURNING_TO_EXPLORE:
                pass
            else:
                print("Error")

            time.sleep(0.1)

    except KeyboardInterrupt as e:
        print("Sending stop")
        robot.set_motor(0.0, 0.0, 0.0, 0.0)
        robot.stop_motor()
        robot.shutdown()

    os._exit(0)
