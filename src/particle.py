import random
from math import cos, sin, sqrt, pi, e
import utils
import transformations


class Particle:
    def __init__(self, x, y, theta, helper):
        self.x = x
        self.y = y
        self.theta = theta
        self.weight = 1.0
        self.map_helper = helper

    def move(self, linear_dt, angular_dt, sigma):
        """
        Cause this particle to experience the same translation and
        rotation that relates to dt, then add noise to the
        resulting position and orientation.
        Check if new particle is not on the obstacle.
        Arguments:
           linear_dt, angular_dt linear and angular velocities based on dt
           sigma variance based on dt
        Returns:
           particle with motion dt applied
        """
        twist = (
            random.gauss(linear_dt * cos(self.theta), sigma),
            random.gauss(linear_dt * sin(self.theta), sigma),
            random.gauss(angular_dt, sigma)
        )
        x = self.x + twist[0]
        y = self.y + twist[1]
        theta = self.theta + twist[2]
        if self.check_within_map(x, y, self.map_helper.get_image_size_pixels()):
            self.x = x
            self.y = y
            self.theta = theta
        else:
            self.x, self.y, self.theta = utils.generate_random_pose(self.map_helper.get_image_width_world(), self.map_helper.get_image_height_world())

    def sense(self, scan_msg):
        """Update the weight of this particle based on a LaserScan message.
        The new weight will be relatively high if the pose of this
        particle corresponds will with the scan, it will be relatively
        low if the pose does not correspond to this scan.

        The algorithm used here is loosely based on the Algorithm in
        Table 6.3 of Probabilistic Robotics Thrun et. al. 2005

        Arguments:
           scan_msg - sensor_msgs/LaserScan object

        Returns:
           None

        """
        min_angle = scan_msg.angle_min
        angle_increment = scan_msg.angle_increment
        ranges = scan_msg.ranges

        ray_probabilities = []
        for idx, value in enumerate(ranges):
            angle = min_angle + angle_increment * idx
            particle_based_scan = self.get_particle_laser_value(angle)
            ray_probability = self._probability_desnsity(value, particle_based_scan)
            ray_probabilities.append(ray_probability)
        self.weight *= sum(ray_probabilities)

    def normalize(self, total_weight):
        self.weight /= total_weight

    def _probability_desnsity(self, _sensor, _particle):
        """
        Gaussian probability density function to assign weights to particles based on laser sensor scan
        f(x|mu,sigma^2) = f(_particle | _sensor, 0.4) = (1/sqrt(2*pi*0.4))*e^((-(_particle-sensor)^2)/(2*0.4))
        weights in particle filter are probabilities
        :param _sensor: laser sensor reading
        :param _particle: reading from particle
        :return: probability based on gaussian model
        """
        sigma_square = 0.4
        numerator = (_particle - _sensor)
        denominator = 2 * sigma_square
        constant_factor = 1.0 / sqrt(2 * pi * sigma_square)
        return constant_factor * e ** (-(numerator * numerator) / denominator)

    def check_within_map(self, world_x, world_y, image_size_pixel):
        pixel_point = transformations.world_to_pixel([world_x, world_y], image_size_pixel)
        return self.map_helper.test_pixel_in_map(pixel_point[0], pixel_point[1])

    def get_particle_laser_value(self, angle):
        # convert particle world location to pixel location
        pixel_points = transformations.world_to_pixel([self.x, self.y], self.map_helper.get_image_size_pixels())
        pixel_theta = transformations.worldtheta_to_pixeltheta(self.theta)
        return self.map_helper.cast_pixel_ray_on_map(pixel_points[0], pixel_points[1], pixel_theta)
