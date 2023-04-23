import numpy as np

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
