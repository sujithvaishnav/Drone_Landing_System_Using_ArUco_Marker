import numpy as np


def clamp(value, lo, hi):

    return max(lo, min(value, hi))


def normalize_angle(angle):

    while angle > np.pi:
        angle -= 2*np.pi

    while angle < -np.pi:
        angle += 2*np.pi

    return angle


def log(msg):

    print(f"[DRONE] {msg}")
