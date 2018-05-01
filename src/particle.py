import random
import numpy as np
from math import cos, sin, sqrt, pi, e
import map_helper
import utils
import transformations


class Particle:
    def __init__(self, x, y, theta, map_path, laser_max, image_scale):
        self.x = x
        self.y = y
        self.theta = theta
        self.weight = 1.0
        self.map_helper = map_helper.MapHelper(map_path, laser_max, image_scale)

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

        xs, ys = self._scan_to_endpoints(scan_msg)
        total_prob = 0
        for i in range(0, len(xs), 10):
            likelihood = self.likelihood_field.get_cell(xs[i], ys[i])
            if np.isnan(likelihood):
                likelihood = 0
            total_prob += np.log(self.laser_z_hit * likelihood +
                                 self.laser_z_rand)

        self.weight *= np.exp(total_prob)

    def _scan_to_endpoints(self, scan_msg):
        """
        Helper method used to convert convert range values into x, y
        coordinates in the map coordinate frame.  Based on
        probabilistic robotics equation 6.32

        """
        theta_beam = np.arange(scan_msg.angle_min, scan_msg.angle_max,
                               scan_msg.angle_increment)
        ranges = np.array(scan_msg.ranges)
        xs = (self.x + ranges * np.cos(self.theta + theta_beam))
        ys = (self.y + ranges * np.sin(self.theta + theta_beam))

        # Clear out nan entries:
        xs = xs[np.logical_not(np.isnan(xs))]
        ys = ys[np.logical_not(np.isnan(ys))]

        # Clear out inf entries:
        xs = xs[np.logical_not(np.isinf(xs))]
        ys = ys[np.logical_not(np.isinf(ys))]
        return xs, ys

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
