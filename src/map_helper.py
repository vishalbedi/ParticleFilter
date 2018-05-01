import cv2
from math import sin, cos, floor


class MapHelper:
    def __init__(self, image_path, laser_max, image_scale):
        self.img = cv2.imread(image_path, 0)
        self.img_height, self.img_width = self.img.shape[:2]
        self.LASER_MAX = laser_max
        self.IMAGE_SCALE = image_scale

    def test_pixel_in_map(self, x, y):
        """
            test if the pixel lies withn the image and return true if
            if the pixel is not toucing any obstacle
        """
        if x < 0 or x > self.img_width:
            return False
        if y < 0 or y > self.img_height:
            return False
        intensity_point = self.img[y, x]
        return intensity_point > 50

    def cast_pixel_ray_on_map(self, x, y, theta):
        """
        Sudo laser scanner for each particle
        Based on angle provided check till what point can ray be traced
        Ray tracing because breshanham is too expensive
        :param x: x co-ordinate of particle in pixels
        :param y: y co-ordinate of particle in pixels
        :param theta: angle of particle in pixels
        :return: max distance for that specific ray on map in meters
        """
        ray_cos = cos(theta)
        ray_sin = sin(theta)
        # get the max distance laser can travel in map
        # LASER_MAX is in meters and IMAGE_SCALE is pixels per meter
        max_laser_dist_in_pixels = int(floor(self.LASER_MAX * self.IMAGE_SCALE)) - 1

        for i in range(max_laser_dist_in_pixels):
            dx = floor(i * ray_cos)
            dy = floor(i * ray_sin)
            # check if there is no obstacle within the ray traced
            if self.test_pixel_in_map(x + dx, y + dy):
                return i / self.IMAGE_SCALE
        return self.LASER_MAX

    def get_image_height_pixels(self):
        return self.img_height

    def get_image_width_pixels(self):
        return self.img_width

    def get_image_height_world(self):
        return self.img_height / self.IMAGE_SCALE

    def get_image_width_world(self):
        return self.img_width / self.IMAGE_SCALE

    def get_image_size_pixels(self):
        return self.img.shape[:2]
