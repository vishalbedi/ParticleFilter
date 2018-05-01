from math import sqrt, pi, e
import particle
import utils


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
        return [p.move(linear_vel_dt, angular_vel_dt, sigma) for p in self.particle_set]

    def sensor_scan(self, msg_scan):
        pass


