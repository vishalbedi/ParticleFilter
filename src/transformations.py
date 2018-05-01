from math import sqrt


MAP_WIDTH = 125.0
MAP_HEIGHT = 43.75
LASER_MAX = 8.0


def world_to_pixel(world_points, image_size):
    world_x, world_y = world_points
    img_h, img_w = image_size
    pixel_points = []
    pixel_points[0] = int(max((world_x / MAP_WIDTH) * img_w, 0))
    if pixel_points[0] > img_w - 1:
        pixel_points[0] = img_w - 1
    pixel_points[1] = int(max((world_y / MAP_HEIGHT) * img_h, 0))
    if pixel_points[1] > img_h - 1:
        pixel_points[1] = img_h
    return pixel_points


def pixel_to_world(pixel_points, image_size):
    img_h, img_w = image_size
    pixel_x, pixel_y = pixel_points
    world_points = []
    world_points[0] = pixel_x/img_w * MAP_WIDTH
    world_points[1] = pixel_y/img_h * MAP_HEIGHT


def dist(point_a, point_b):
    return sqrt((point_a[0] - point_b[0]) ** 2 + (point_a[1] - point_b[1]) ** 2)
