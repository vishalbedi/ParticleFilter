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
import transformations



class Mapper(tk.Frame):

    def __init__(self, image_path, *args, **kwargs):
        tk.Frame.__init__(self, *args, **kwargs)
        self.master.title("I'm the map!")

        # makes a grey-scale image filled with 50% grey pixels
        self.themap = Image.open(image_path)
        self.mapimage = ImageTk.PhotoImage(self.themap)
        (MAPWIDTH, MAPHEIGHT) = self.themap.size
        rospy.loginfo((MAPWIDTH, MAPHEIGHT))
        self.master.minsize(width=MAPWIDTH, height=MAPWIDTH)

        # this gives us directly memory access to the image pixels:
        self.mappix = self.themap.load()
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
        self.odem_msg = {}
        self.MAPHEIGHT = MAPHEIGHT
        self.MAPWIDTH = MAPWIDTH

    def update_image(self):
        self.mapimage = ImageTk.PhotoImage(self.themap)
        self.canvas.create_image(self.MAPWIDTH / 2, self.MAPHEIGHT / 2, image=self.mapimage)

    def map_update(self, points):
        # note this function just lowers the odds in a random area...
        for point in points:
            rospy.loginfo(point)
            
            draw = ImageDraw.Draw(self.themap)
            draw.ellipse((point[0]-1, point[1]-1, point[0]+1, point[1]+1), fill=(255,0,0,255))

        # this puts the image update on the GUI thread, not ROS thread!
        # also note only one image update per scan, not per map-cell update
        self.after(0, self.update_image)

    def laser_callback(self, msg):
        self.pf.filter(sensor_msg=msg, twist=self.odem_msg.twist)
        particles = self.pf.particle_set
        particle_points = []
        for p in particles:
            x, y = transformations.world_to_pixel((p.x, p.y),(self.MAPHEIGHT,self.MAPWIDTH ) )
            particle_points.append([x, y])
        self.map_update(particle_points)

    def odem_callback(self, msg):
        self.odem_msg = msg


def main():
    rospy.init_node("mapper")
    rospy.loginfo("started")
    root = tk.Tk()
    IMAGE_WIDTH = 2000
    IMAGE_HEIGHT = 700
    m = Mapper(image_path="/home/stu1/s6/vgb8777/catkin_ws/src/localization/map/map.png", master=root, height=IMAGE_HEIGHT, width=IMAGE_WIDTH)
    rospy.Subscriber("/r1/kinect_laser/scan", LaserScan, m.laser_callback)
    rospy.Subscriber("/r1/odom", Odometry, m.odem_callback)
    rospy.Timer(rospy.Duration(0.01), m.pf.clustering)
    rospy.Timer(rospy.Duration(0.2), root.mainloop())


if __name__ == "__main__":
    main()
