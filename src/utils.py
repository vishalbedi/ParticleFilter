import random
from math import pi


def generate_random_pose(MAP_WIDTH, MAP_HEIGHT):
    x = random.uniform(0, MAP_WIDTH)
    y = random.uniform(0, MAP_HEIGHT)
    theta = random.uniform(0, 2 * pi)
    return x, y, theta


