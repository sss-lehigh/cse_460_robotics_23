from apis import *
import time

if __name__ == "__main__":
   
    robot = LocalRobot()
    position = Position("192.168.0.207", "192.168.0.172", 307)
    camera = Camera(raspberrypi=True)

    K1 = 50.0
    K2 = 9.0

    ram_time = 1.0
    ramming = False
    start = time.time()

    try:
        while True:

            x_t, angle = position.get()

            blob = camera.get_largest_blob()

            w = 0.0
            v = 0.0

            if blob is not None and not ramming:
                start = time.time()
                if abs((320.0 - blob[0])) < 20.0:
                    print("Ramming it", 320.0 - blob[0])
                    w = 0.0
                    v = 1500.0
                    ramming = True
                else:
                    sign = 1.0
                    if 320.0 - blob[0] < 0.0:
                        sign = -1.0
                    w = sign * 800.0
                    print("Rotating",w)
            elif ramming and time.time() - start < ram_time:
                w = 0.0
                v = 1500.0
            elif time.time() - start >= ram_time:
                ramming = False
            
            #w = K2 * delta_angle
            u = np.array([v - w, v + w])

            robot.set_motor(u[0], u[0], u[1], u[1])

            time.sleep(0.1)

    except KeyboardInterrupt as e:
        print("Sending stop")
        
    robot.set_motor(0.0, 0.0, 0.0, 0.0)
    robot.stop_motor()
    robot.shutdown()

    os._exit(0)