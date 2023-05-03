from apis import *

if __name__ == "__main__":
   
    position = Position("192.168.0.207", "192.168.0.172", 307)

    detector = ObstacleDetector()

    x_t, angle = position.get()

    if detector.sense():
        print(detector.get_loc(x_t, angle))