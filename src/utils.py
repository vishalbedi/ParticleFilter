import random
from math import pi

FOCUS_POINT = [(8, -0.5), (-12.0, 12.0), (-18.4, -8.9), (10.8, 12.7), (-54.5, 7.6), (8, -1.5)]


def generate_random_pose():
    index = random.uniform(0,5)
    deveation = 10
    x = random.gauss(FOCUS_POINT[index], deveation)
    y = random.uniform(FOCUS_POINT[index], deveation)
    theta = random.uniform(0, 2 * pi)
    return x, y, theta


def radians(degree):
    return degree * pi / 180
