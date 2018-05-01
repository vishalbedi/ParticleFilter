from math import sqrt, pi, e
import particle
import utils
import numpy as np


class ParticleFilter:
    def __init__(self, map_width,  map_height, image_path, laser_max, pixels_per_meter, num_of_particles=100, dt=0.1):
        self.PARTICLE_COUNT = num_of_particles
        self.NOISE = int(self.PARTICLE_COUNT * .10)
        self.dt = dt
        self.centroid = (0, 0, 0)
        self.MAP_WIDTH = map_width
        self.MAP_HEIGHT = map_height
        self.image_path = image_path
        self.laser_max = laser_max
        self.pixels_per_meter = pixels_per_meter
        self.particle_set = [self.generate_random_particle() for _ in range(self.PARTICLE_COUNT)]

    def generate_random_particle(self):
        """
        Generate a particle randomly within the bounds of map
        :return: a new particle instance
        """
        x, y, theta = utils.generate_random_pose(self.MAP_WIDTH, self.MAP_HEIGHT)
        return particle.Particle(x, y, theta, self.image_path, self.laser_max, self.pixels_per_meter)

    def motion_sense(self, twist):
        """
        We update motion based on twist instead of pose
        thus instead of calculating pose1 - pose2
        we assume small change to twist with respect to dt
        this does not updates weight
        :param twist: current angular and linear velocity
        :return: new particle set with motion applied
        """
        linear_vel = twist.linear.x
        angular_vel = twist.angular.z

        linear_vel_dt = linear_vel * self.dt
        angular_vel_dt = angular_vel * self.dt
        sigma = (sqrt(linear_vel ** 2 + angular_vel ** 2) / 4.0) * self.dt
        # move every particle by motion that happens in time dt
        self.particle_set = [p.move(linear_vel_dt, angular_vel_dt, sigma) for p in self.particle_set]

    def sensor_scan(self, msg_scan):
        for p in self.particle_set:
            p.sense(msg_scan)

        weights = [p.weight for p in self.particle_set]
        total_weight = sum(weights)
        for p in self.particle_set:
            p.normalize(total_weight)

    def _select_particles_based_on_weight(self, num_of_particles):
        weights = [p.weight for p in self.particle_set]
        particles = self.particle_set

        np_weights = np.array(weights, dtype=np.double)
        cdf_weights = np_weights.cumsum()
        cdf_weights /= cdf_weights [-1]
        # create a array randomly whose values are between [0-1] and take particles from
        # particle array whose weights are close my match of this array.
        idxs = []
        uniform_samples = np.random.random(num_of_particles)
        sorted_weights = np.sort(cdf_weights)
        for sample in uniform_samples:
            for i, weight in reversed(list(enumerate(sorted_weights))):
                if sample> weight:
                    continue
                else:
                    idxs.append(i)
                    break

        return map(lambda ii: particles[ii], idxs)

    def resample(self):
        """
        Calculate the variance of weights and resample if the variance is less than 80% of total particles
        :return:
        """
        sum_of_weight_squares = sum([p.weight**2 for p in self.particle_set])
        variance_in_weights = 1.0/sum_of_weight_squares
        # select particles randomly from the current particle set
        if variance_in_weights < self.PARTICLE_COUNT * .80:
            particles = self._select_particles_based_on_weight(self.PARTICLE_COUNT - self.NOISE) + [self.generate_random_particle() for _ in range(self.NOISE)]
            self.particle_set = particles
            for p in self.particle_set:
                p.weight = 1.0

    def filter(self, sensor_msg, twist):
        self.sensor_scan(sensor_msg)
        self.resample()
        self.motion_sense(twist)

    def clustering(self):
        pass