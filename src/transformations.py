from math import sqrt, cos, sin, atan2

WORLD_MAP_WIDTH = 125.0
WORLD_MAP_HEIGHT = 43.75
LASER_MAX = 10.0


def world_to_pixel(world_points, image_size):
    world_x, world_y = world_points
    img_h, img_w = image_size
    pixel_points = [0,0]
    pixel_points[0] = int(max(abs(world_x / WORLD_MAP_WIDTH) * img_w, 0)) * (world_x/abs(world_x))
    if pixel_points[0] > img_w - 1:
        pixel_points[0] = img_w - 1
    pixel_points[1] = int(max(abs(world_y / WORLD_MAP_HEIGHT) * img_h, 0)) * (world_y/abs(world_y))
    if pixel_points[1] > img_h - 1:
        pixel_points[1] = img_h
    pixel_points[1] = pixel_points[1]
    pixel_points[0] = img_w/2 + pixel_points[0]
    pixel_points[1] = img_h/2 - pixel_points[1]
    return pixel_points


def pixel_to_world(pixel_points, image_size):
    img_h, img_w = image_size
    pixel_x, pixel_y = pixel_points
    world_points = [0,0]
    world_points[0] = (pixel_x / img_w) * WORLD_MAP_WIDTH
    world_points[1] = (pixel_y / img_h) * WORLD_MAP_HEIGHT
    world_points[0] = world_points[0] - WORLD_MAP_WIDTH / 2
    world_points[1] = WORLD_MAP_HEIGHT / 2 - world_points[1]
    return world_points


def worldtheta_to_pixeltheta(world_theta):
    x = 1 * cos(world_theta)
    y = 1 * sin(world_theta)
    # since the y axis is inverted in the map the angle will be different
    return atan2(-y, x)


def dist(point_a, point_b):
    return sqrt((point_a[0] - point_b[0]) ** 2 + (point_a[1] - point_b[1]) ** 2)
