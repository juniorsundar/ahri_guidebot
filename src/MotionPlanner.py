#!/usr/bin/python
import intera_interface
import rospy
import copy 
import random
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from shapely.geometry import Point
from shapely.geometry.polygon import Polygon

from geometry_msgs.msg import Pose, Point, Quaternion

rospy.init_node('MotionPlanner')
g_limb = intera_interface.Limb('right')
g_limb.set_joint_position_speed(.1)
workspace = np.array([[0,0.7],[-0.7,0.7],[0,0.2]])
graph = []
quat = Quaternion(0.704238785359,0.709956638597,-0.00229009932359,0.00201493272073)

class Node:
    def __init__(self,pose,angles):
        self.pose = pose
        self.jointAngles = dict_to_array(angles)
        self.neighbours = []
        self.weight = []

    def distanceTo(self,newNode):
        return np.linalg.norm(newNode.jointAngles - self.jointAngles)

    def Link(self,index):
        self.neighbours.append(index)

    def eDist(self,newNode):
        ax=self.pose.position.x
        ay=self.pose.position.y
        az=self.pose.position.z
        bx=newNode.pose.position.x
        by=newNode.pose.position.y
        bz=newNode.pose.position.z
        return np.linalg.norm(np.array([ax,ay,az])-np.array([bx,by,bz]))

def dict_to_array(inputDict):
    array = []
    array.append(inputDict['right_j0'])
    array.append(inputDict['right_j1'])
    array.append(inputDict['right_j2'])
    array.append(inputDict['right_j3'])
    array.append(inputDict['right_j4'])
    array.append(inputDict['right_j5'])
    array.append(inputDict['right_j6'])
    return np.asarray(array)

def Nearest(newNode):
    global graph
    closeAngIdx = []
    closeIdx = 0
    angDist = .5
    euDist = .4
    for i in range(0,len(graph)):
        if graph[i].distanceTo(newNode) < angDist:
            angDist = graph[i].distanceTo(newNode)
            closeAngIdx.append(i)
    for i in range(0,len(graph)):
        if graph[i].eDist(newNode) < euDist:
            euDist = graph[i].eDist(newNode)
            closeIdx = i
    return closeIdx

def Exists(newNode):
    global graph
    exists = False
    for nodes in graph:
        if newNode.distanceTo(nodes) < 0.001:
            exists = True
            break
    return exists

def GenerateRandom():
    global workspace
    global graph
    global quat

    x = (workspace[0][1]-workspace[0][0])*np.random.random_sample() + workspace[0][0]
    y = (workspace[1][1]-workspace[1][0])*np.random.random_sample() + workspace[1][0]
    z = (workspace[2][1]-workspace[2][0])*np.random.random_sample() + workspace[2][0]

    tPoint = Point(x,y,z)
    tPose = Pose()

    tPose.position = copy.deepcopy(tPoint)
    tPose.orientation = copy.deepcopy(quat)
    angles_limb = g_limb.ik_request(tPose, "right_hand")
    while angles_limb is False:
        x = (workspace[0][1]-workspace[0][0])*np.random.random_sample() + workspace[0][0]
        y = (workspace[1][1]-workspace[1][0])*np.random.random_sample() + workspace[1][0]
        z = (workspace[2][1]-workspace[2][0])*np.random.random_sample() + workspace[2][0]

        tPoint = Point(x,y,z)
        tPose = Pose()

        tPose.position = copy.deepcopy(tPoint)
        tPose.orientation = copy.deepcopy(quat)
        angles_limb = g_limb.ik_request(tPose, "right_hand")
    
    tNode = Node(tPose,angles_limb)

    if len(graph)>0:
        while Exists(tNode):
            tNode = GenerateRandom()

    return tNode

def RRT():
    global graph, quat
    start_point = Point(0.0,0.0,0.2)

    start = Pose()
    start.position = copy.deepcopy(start_point)
    start.orientation = copy.deepcopy(quat)

    target_joint_angles = g_limb.ik_request(start, "right_hand")
    start = Node(start,target_joint_angles)
    graph.append(start)
    for i in range(0,500):
        XNew = GenerateRandom()
        XNearestIdx = Nearest(XNew)
        graph.append(XNew)
        graph[i+1].Link(XNearestIdx)
        graph[XNearestIdx].Link(i+1)

    fig = plt.figure()
    ax = fig.gca(projection='3d')
    for i in range(0,len(graph)):
        if i == 0:
            ax.scatter(graph[i].pose.position.x,graph[i].pose.position.y,graph[i].pose.position.z,c='g',marker='o')
        else:
            ax.scatter(graph[i].pose.position.x,graph[i].pose.position.y,graph[i].pose.position.z,c='r',marker='x')
        for j in range(0,len(graph[i].neighbours)):
            Lx = np.array([graph[i].pose.position.x,graph[graph[i].neighbours[j]].pose.position.x])
            Ly = np.array([graph[i].pose.position.y,graph[graph[i].neighbours[j]].pose.position.y])
            Lz = np.array([graph[i].pose.position.z,graph[graph[i].neighbours[j]].pose.position.z])
            ax.plot(Lx,Ly,Lz,c='k')
    plt.show()

if __name__ == "__main__":
    RRT()