from math import sqrt
import particle
import utils
import numpy as np
import transformations
import random
import rospy
import map_helper

class ParticleFilter:
    def __init__(self, map_width,  map_height, image_path, pixels_per_meter, num_of_particles=100, dt=0.1):
        self.PARTICLE_COUNT = num_of_particles
        self.NOISE = int(self.PARTICLE_COUNT * .10)
        self.dt = dt
        self.centroid = (0, 0, 0)
        self.WORLD_MAP_WIDTH = map_width
        self.WORLD_MAP_HEIGHT = map_height
        self.image_path = image_path
        self.laser_max = 10
        self.pixels_per_meter = pixels_per_meter
        self.particle_set = [self.generate_random_particle() for _ in range(self.PARTICLE_COUNT)]
        self.isLocalized = False
        self.isClustering = False
        self.CLUSTER_SIZE = 15
        self.NEIGHBOR_THRESHOLD = 0.2
        self.helper = map_helper.MapHelper(map_path, laser_max, image_scale)

    def generate_random_particle(self):
        """
        Generate a particle randomly within the bounds of map
        :return: a new particle instance
        """
        x, y, theta = utils.generate_random_pose(self.WORLD_MAP_WIDTH, self.WORLD_MAP_HEIGHT)
        return particle.Particle(x, y, theta, self.image_path, self.laser_max, self.pixels_per_meter, self.helper)

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
        self.laser_max = sensor_msg.range_max
        # don't update if clustering
        if self.isClustering:
            return
        self.sensor_scan(sensor_msg)
        self.resample()
        self.motion_sense(twist)

    def clustering(self,event):
        """
        Perform K means clustering to get the centroid of the particles
        do not perform any update while clustering
        :return:
        """
        while True:
            self.isClustering = True
            particles = self.particle_set
            particles = sorted(particles, key=lambda k: (k.x, k.y, k.theta))
            visited = set()
            clusters = {}
            cluster_count = 0
            # create clusters
            for i in range(self.PARTICLE_COUNT):
                if i not in visited:
                    queue = [i]
                    cluster_count +=1
                    clusters[cluster_count] = []
                    while len(queue):
                        current_particle_index = queue.pop()
                        current_particle = particles[current_particle_index]
                        clusters[cluster_count].append(current_particle)
                        end_particle_index = current_particle_index + self.CLUSTER_SIZE
                        if end_particle_index > self.PARTICLE_COUNT:
                            end_particle_index = self.PARTICLE_COUNT
                        cluster_distances = []
                        for neighbour_index in range(current_particle_index, end_particle_index):
                            if neighbour_index not in visited:
                                neighbour_particle = particles[neighbour_index]
                                cluster_distance = (transformations.dist((current_particle.x, current_particle.y), (neighbour_particle.x, neighbour_particle.y)), neighbour_index)
                                cluster_distances.append(cluster_distance)
                        cluster_distances.sort(lambda a, b: cmp(a[0], b[0]))
                        for distance, index in cluster_distances:
                            if distance < self.NEIGHBOR_THRESHOLD:
                                visited.add(neighbour_index)
                                queue.append(neighbour_index)
                            else:
                                break
            biggest_cluster_size = 0
            for cluster in clusters.values():
                if len(cluster) > biggest_cluster_size:
                    biggest_cluster_size = len(cluster)
                    biggest_cluster = cluster
            cluster_coordinates = [(p.x, p.y) for p in biggest_cluster]
            cluster_coordinates = np.array(cluster_coordinates)
            centroid = np.mean(cluster_coordinates, axis=0)
            variance = np.var(cluster_coordinates, axis=0)
            self.centroid = (centroid[0],centroid[1], biggest_cluster[random.randint(0,biggest_cluster_size-1)].theta)
            #rospy.loginfo("Centroid ")
            #rospy.loginfo(self.centroid)

            if biggest_cluster_size > self.PARTICLE_COUNT * .75:
                self.isLocalized = True
            else:
                self.isLocalized = False
            self.isClustering = False
