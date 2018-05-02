import random
from math import pi


def generate_random_pose(MAP_WIDTH, MAP_HEIGHT):
    x = random.uniform(-MAP_WIDTH/2, MAP_WIDTH/2)
    y = random.uniform(-MAP_HEIGHT/2, MAP_HEIGHT/2)
    theta = random.uniform(0, 2 * pi)
    return x, y, theta


def radians(degree):
    return degree * pi / 180
