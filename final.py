from apis import *

# bounds

bounds = [[-4.39, 3.73],
          [-4.39, -2.14],
          [6.0, -2.14],
          [6.0, 3.73]]

if __name__ == "__main__":
   
    robot_id = 207

    robot = LocalRobot()
    position = Position("192.168.0." + str(robot_id), "192.168.0.4", robot_id)

    x_d = np.array([0., 0.])
    
    try:
        while True:
            x_t, angle = position.get()

            print(x_t)

            K1 = 1000
            K2 = 1500

            dist, desired = dist_and_angle(x_d, x_t)

            delta_angle = angle_diff(desired, angle)

            v = K1 * dist
            w = K2 * delta_angle
            u = np.array([0.,0.])
            #u = np.array([v - w, v + w])

            robot.set_motor(u[0], u[0], u[1], u[1])

    except (Exception, KeyboardInterrupt) as e:
        print(type(e))

    robot.stop_motor()
    robot.shutdown()
