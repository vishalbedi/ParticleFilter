#!/usr/bin/python
'''
  Some Tkinter/PIL code to pop up a window with a gray-scale
  pixel-editable image, for mapping purposes.  Does not run
  until you fill in a few things.

  Does not do any mapping.

  Z. Butler, 3/2016, updated 3/2018
'''

import Tkinter as tk
from PIL import Image, ImageDraw
import ImageTk
import rospy
from sensor_msgs.msg import LaserScan
from p2os_msgs.msg import SonarArray
from nav_msgs.msg import Odometry
import sys
import paricle_filter
from geometry_msgs.msg import Twist
import transformations
import safewander
import astar
import random
from node import Node
from math import pi


class Mapper(tk.Frame):
    def __init__(self, image_path, *args, **kwargs):
        self.goal = []
        tk.Frame.__init__(self, *args, **kwargs)
        self.master.title("I'm the map!")
        self.baseImg = Image.open(image_path).convert("RGBA")
        # makes a grey-scale image filled with 50% grey pixels
        self.themap = Image.open(image_path).convert("RGBA")
        self.mapimage = ImageTk.PhotoImage(self.themap)
        (MAPWIDTH, MAPHEIGHT) = self.themap.size
        self.master.minsize(width=MAPWIDTH, height=MAPWIDTH)
        # keeping the odds separately saves one step per cell update:
        WORLD_MAP_WIDTH = 125.0
        WORLD_MAP_HEIGHT = 43.75
        self.pf = paricle_filter.ParticleFilter(WORLD_MAP_WIDTH, WORLD_MAP_HEIGHT, image_path, 16)
        self.canvas = tk.Canvas(self, width=MAPWIDTH, height=MAPHEIGHT)
        self.laser_scan = []
        self.sonar_scan = []
        self.map_on_canvas = self.canvas.create_image(MAPWIDTH / 2, MAPHEIGHT / 2, image=self.mapimage)
        self.canvas.pack()
        self.pack()
        self.odem_msg = Odometry()
        self.MAPHEIGHT = MAPHEIGHT
        self.MAPWIDTH = MAPWIDTH
        self.wanderer = safewander.SafeWander(self._get_wanderer_coordinates())
        self.pathplanner = None

    def _get_wanderer_coordinates(self):
        coordinates = []
        for i in range(20):
            x = random.randint(-10, 10)
            y = random.randint(-10, 10)
            p = (x, y)
            coordinates.append(p)
        return coordinates

    def set_goal(self, x, y, theta):
        self.goal = [x, y, theta]

    def update_image(self):
        rospy.loginfo("plotting points")
        self.mapimage = ImageTk.PhotoImage(self.themap)
        self.canvas.create_image(self.MAPWIDTH / 2, self.MAPHEIGHT / 2, image=self.mapimage)
        self.themap = self.baseImg

    def map_update(self, points):
        # note this function just lowers the odds in a random area...
        for point in points:
            draw = ImageDraw.Draw(self.themap)
            draw.ellipse((point[0] - 1, point[1] - 1, point[0] + 1, point[1] + 1), fill=(255, 0, 0, 255))
        # this puts the image update on the GUI thread, not ROS thread!
        # also note only one image update per scan, not per map-cell update
        self.after(0, self.update_image)

    def laser_callback(self, msg):
        rospy.loginfo("Laser CallBack")
        self.pf.filter(sensor_msg=msg, twist=self.odem_msg.twist)
        self.wanderer.islocalized = self.pf.isLocalized
        particles = self.pf.particle_set
        particle_points = []
        for p in particles:
            x, y = transformations.world_to_pixel((p.x, p.y), (self.MAPHEIGHT, self.MAPWIDTH))
            particle_points.append([x, y])
        if self.pf.isLocalized:
            start = Node(self.pf.centroid[0], self.pf.centroid[1], self.pf.centroid[2])
            end = Node(self.goal[0], self.goal[1], self.goal[2])
            self.pathplanner = astar.PathPlanner(start, pi, end)
            self.pathplanner.plan()
        self.map_update(particle_points)

    def odem_callback(self, msg):
        rospy.loginfo("Odem CallBack")
        self.odem_msg = msg


def main(x, y, theta):
    rospy.init_node("mapper")
    rospy.loginfo("started")
    rospy.Rate(10)
    root = tk.Tk()
    IMAGE_WIDTH = 2000
    IMAGE_HEIGHT = 700
    m = Mapper(image_path="/home/stu1/s6/vgb8777/catkin_ws/src/localization/map/map.png", master=root,
               height=IMAGE_HEIGHT, width=IMAGE_WIDTH)
    m.set_goal(x, y, theta)
    rospy.Subscriber('/r1/odom', Odometry, m.wanderer.odom_callback)
    rospy.Subscriber('/r1/kinect_laser/scan', LaserScan, m.wanderer.laser_callback)
    rospy.Subscriber('/r1/pseudosonar/scan', LaserScan, m.wanderer.sonar_callback)
    rospy.Subscriber("/r1/kinect_laser/scan", LaserScan, m.laser_callback)
    rospy.Subscriber("/r1/odom", Odometry, m.odem_callback)
    rospy.Timer(rospy.Duration(0.01), m.pf.clustering)
    rospy.Timer(rospy.Duration(0.1), m.wanderer.travel)
    rospy.Timer(rospy.Duration(0.2), root.mainloop())


if __name__ == "__main__":
    try:
        if not len(sys.argv) == 4:
            rospy.loginfo("Usage: rosrun localization mapper.py <x> <y> <theta>")
        else:
            main(sys.argv[1], sys.argv[2], sys.argv[3])
    except rospy.ROSInterruptException:
        pass
