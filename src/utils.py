import random
from math import pi


def generate_random_pose(focus_x, focus_y):
    deveation = 20
    x = random.gauss(focus_x, deveation)
    y = random.uniform(focus_y, deveation)
    theta = random.uniform(0, 2 * pi)
    return x, y, theta


def radians(degree):
    return degree * pi / 180
