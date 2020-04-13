#!/usr/bin/python
import intera_interface
import rospy
import copy 
import time
import random
import numpy as np
import matplotlib.pyplot as plt
from shapely.geometry import LineString, Polygon
from shapely.geometry import Point as Points
from shapely import affinity
import os
from geometry_msgs.msg import Pose, Point, Quaternion
from ahri_guidebot.msg import Coordinates

rospy.init_node('WayPoints',anonymous=True)
pub = rospy.Publisher('waypoint',Coordinates,queue_size=10)
workspace = np.array([[0,0.7],[-0.7,0.7],[0,0.2]])
(0.704238785359,0.709956638597,-0.00229009932359,0.00201493272073)
start_point_robot = Point(0.0,0.0,0.1)
start_point_human = Point(0.7,0,0.1)
end_point_human = Point()

obs_c = np.array([[-0.5,0.25455],[-0.3,0.54091],[0,0.38182],[0.35,0.54091],[0.4,0.25455]])
obs_w = 0.05
obs_h = 0.03

o = Polygon([(0.3,0.3), (0.3,0.4), (0.1,0.4), (0.1,0.3)])
o1 = Polygon([(-0.3,0.3), (-0.3,0.4), (-0.1,0.4), (-0.1,0.3)])
obstacles = [affinity.scale(o,yfact=-2,origin=(0,0.35)),affinity.scale(o1,yfact=-2,origin=(0,0.35))]

##x: 0-28 | y: 0-22
##x: -0.7-0.7 | y: 0-0.7
##obs: 1(4,8) | 2(8,17) | 3(14,12) | 4(21,17) | 5(22,8)

def intersects(point):
    point = Points(point.y,point.x)
    output = False
    for k in obstacles:
        if k.contains(point):
            output = True
    return output

def callback(data)
    if intersects(data):
        publishing = Coordinates()
        publishing.start_point = data
        publishing.end_point = end_point_human
        pub.publish(publishing)
        time.sleep(5)

def main():
    rospy.Subscriber('camera_input',Point,callback)
    rospy.spin()

if __name__ == "__main__":
    end_point_human = Point(float(raw_input('x: ')),float(raw_input('y: ')),float(raw_input('z: ')))
    main()