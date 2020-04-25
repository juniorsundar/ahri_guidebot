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

rospy.init_node('WayPoints',anonymous=False)
pub = rospy.Publisher("waypoint",Coordinates,queue_size=10)
workspace = np.array([[0,0.7],[-0.7,0.7],[0,0.2]])
start_point_robot = Point(0.0,0.0,0.1)
shoulder_point_human = Point(0.7,0,0.1)
end_point_human = Point()

##HARDCODED OBSTACLES
o = Polygon([(0.3,0.3), (0.3,0.4), (0.1,0.4), (0.1,0.3)])
o1 = Polygon([(-0.3,0.3), (-0.3,0.4), (-0.1,0.4), (-0.1,0.3)])
obstacles = [affinity.scale(o,yfact=-2,origin=(0,0.35)),affinity.scale(o1,yfact=-2,origin=(0,0.35))]
obstacles[0] = affinity.scale(obstacles[0],xfact = 2, origin=(0.2,0.35))
obstacles[1] = affinity.scale(obstacles[1],xfact = 2, origin=(-0.2,0.35))

def callback(data):
    publishing = Coordinates()
    publishing.start_point = data
    publishing.end_point = end_point_human
    pub.publish(publishing)
    time.sleep(1)

def main():
    rospy.Subscriber('camera_input',Point,callback)
    rospy.spin()

if __name__ == "__main__":
    end_point_human = Point(0.1,-0.6,0.0)  ##CHANGE THIS TO MODIFY THE GOAL FOR THE HUMAN HAND
    main()
