from apis import *
import time

if __name__ == "__main__":
   
    robot = RemoteRobot("192.168.0.207")
    position = Position("192.168.0.139", "192.168.0.172", 307)
    grid = Discretization(-3.3, -.26, -.46, 2.63)

    x_d = np.array([0., 0.])
    
    print("Here")

    x_t, angle = position.get()

    print("Here")


    curr_node, _ = grid.get_nearest_node(x_t)

    path = grid.get_path(curr_node, 30)

    K1 = 1000
    K2 = 1500

    node_idx = 1

    start = time.time()

    init = np.copy(x_t)

    init = np.zeros(2)
    slope = np.zeros(2)

    if node_idx < len(path):
        init = x_t
        slope = path[node_idx] - init
    else:
        slope = np.zeros(2)
        init = path[-1]

    factor = 2.0

    robot.stop_motor()

    try:
        while True:

            x_t, angle = position.get()

            curr_time = time.time()

            if (curr_time - start) / factor > 1.0:
                start = curr_time
                node_idx += 1
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

            time.sleep(0.1)

    except KeyboardInterrupt as e:
        print("Sending stop")
        robot.set_motor(0.0, 0.0, 0.0, 0.0)
        robot.stop_motor()
        robot.shutdown()

    os._exit(0)