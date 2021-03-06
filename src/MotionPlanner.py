#!/usr/bin/python
import intera_interface
import rospy
import copy 
import random
import numpy as np
import matplotlib
matplotlib.use('Agg')
from matplotlib import pyplot as plt
from shapely.geometry import LineString, Polygon
from shapely.geometry import Point as Points
from shapely import affinity
import os
import datetime

from geometry_msgs.msg import Pose, Point, Quaternion
from ahri_guidebot.msg import Coordinates

rospy.init_node('MotionPlanner',anonymous=False)
g_limb = intera_interface.Limb('right')
g_limb.set_joint_position_speed(.1)
workspace = np.array([[0,0.7],[-0.7,0.7],[0,0.1]])
graph = []
quat = Quaternion(0.704238785359,0.709956638597,-0.00229009932359,0.00201493272073)
start_point_robot = Point(0.0,0.0,0.1)
start_point_human = Point(0.7,0,0.1)

##HARDCODED OBSTACLES
o = Polygon([(0.3,0.3), (0.3,0.4), (0.1,0.4), (0.1,0.3)])
o1 = Polygon([(-0.3,0.3), (-0.3,0.4), (-0.1,0.4), (-0.1,0.3)])
obstacles = [affinity.scale(o,yfact=-1,origin=(0,0.35)),affinity.scale(o1,yfact=-1,origin=(0,0.35))]

class Node:
    def __init__(self,pose,angles):
        self.pose = pose
        self.angles = angles
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

class Vertex:
    def __init__(self,node):
        self.f = 0.0
        self.g = 0.0
        self.h = 0.0
        self.previous = 0
        self.Node = node

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

def Nearest(newNode,NodeIndex):
    global graph
    closeAngIdx = []
    closeIdx = []
    angDist = .6
    euDist = .1
    for i in range(1,len(graph)):
        if i != NodeIndex and i not in graph[NodeIndex].neighbours and len(graph[NodeIndex].neighbours) < 5:
            if graph[i].eDist(newNode) < euDist:
                # angDist = graph[i].distanceTo(newNode)
                closeIdx.append(i)
    for i in closeIdx:
        if graph[i].distanceTo(newNode) < angDist:
            # euDist = graph[i].eDist(newNode)
            closeAngIdx.append(i)
    return closeAngIdx

def Exists(newNode):
    global graph
    exists = False
    for nodes in graph:
        if newNode.distanceTo(nodes) < 0.05:
            exists = True
            break
    return exists

def GenerateRandom():
    global workspace
    global graph
    global quat
    global g_limb

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

def isInObstacle(newNode):
    x = float(newNode.pose.position.y)
    y = float(newNode.pose.position.x)
    isIn = False
    for i in range(0,len(obstacles)):
        c = affinity.scale(obstacles[i],xfact=1.5,yfact=1.5)
        c = c.bounds
        if x > c[0] and x < c[2] and y > c[1] and y < c[3]:
            isIn = True
    return isIn

##OUR CONTRIBUTION
def humanCost(current, neighbour):
    global graph, start_point_human, obstacles
    cost = False
    p3 = Points(start_point_human.y,start_point_human.x)
    p1 = Points(current.Node.pose.position.y,current.Node.pose.position.x)
    p2 = Points(neighbour.Node.pose.position.y,neighbour.Node.pose.position.x)
    poly = Polygon([p1,p2,p3])
    for k in obstacles:
        c = affinity.scale(k,xfact=1.5,yfact=1.5)
        if c.intersects(poly):
            cost = True
    return cost

def RRT(): #more like PRM
    global graph, quat, start_point_robot

    start = Pose()
    start.position = copy.deepcopy(start_point_robot)
    start.orientation = copy.deepcopy(quat)

    target_joint_angles = g_limb.ik_request(start, "right_hand")
    start = Node(start,target_joint_angles)
    graph.append(start)
    for i in range(0,2000):
        XNew = GenerateRandom()
        while isInObstacle(XNew):
            XNew = GenerateRandom()
        graph.append(XNew)
    for i in range(len(graph)):
        XNearestIdx = Nearest(graph[i],i)
        for j in XNearestIdx:
            graph[i].Link(j)
            graph[j].Link(i)
    os.system('clear')
    global x
    if x:
        fig = plt.figure()
        for i in range(len(graph)):
            if i == 0:
                plt.scatter(graph[i].pose.position.x,graph[i].pose.position.y,c='g',marker='o')
            else:
                plt.scatter(graph[i].pose.position.x,graph[i].pose.position.y,c='r',marker='x')
            for j in range(len(graph[i].neighbours)):
                Lx = np.array([graph[i].pose.position.x,graph[graph[i].neighbours[j]].pose.position.x])
                Ly = np.array([graph[i].pose.position.y,graph[graph[i].neighbours[j]].pose.position.y])
                Lz = np.array([graph[i].pose.position.z,graph[graph[i].neighbours[j]].pose.position.z])
                plt.plot(Lx,Ly,c='k')
            for c in obstacles:
                y,x = c.exterior.xy
                plt.plot(x,y)
        fig.savefig('src/ahri_guidebot/plot.png')
        plt.show()
    
def reconstruct_path(current):
    total_path = [current]
    while total_path[-1] != total_path[-1].previous:
        current = current.previous
        total_path.append(current)
    for i in range(0,len(total_path)):
        total_path[i] = total_path[i].Node
    return total_path
    
def A_Star(start,goal):
    global graph
    graph2 = []
    for node in graph:
        graph2.append(Vertex(node))

    for i in range(0,len(graph2)):
        graph2[i].f = 1000
        graph2[i].g = 1000

    openSet = [graph2[start]]
    closedSet = []

    openSet[0].f = openSet[0].Node.eDist(graph2[goal].Node)
    openSet[0].g = 0
    openSet[0].previous = openSet[0]
    
    while len(openSet)>0:
        winner = 0
        
        for i in range(len(openSet)):
            if openSet[i].f < openSet[winner].f:
                winner = i
        current = openSet[winner]

        if current.Node.eDist(graph2[goal].Node) < 0.01:
            return reconstruct_path(current)
        
        openSet.pop(winner)
        closedSet.append(current)
        
        for ni in current.Node.neighbours:
            neighbour = graph2[ni]

            tentG = current.g + current.Node.eDist(neighbour.Node) 
            if not humanCost(current, neighbour): ##THIS IS THE EVALUATION OF WHETHER THE TRANSITION IS SAFE
                if tentG < neighbour.g:
                    neighbour.previous = current
                    neighbour.g = tentG
                    neighbour.f = neighbour.g + current.Node.eDist(graph2[goal].Node)
                    if neighbour not in openSet:
                        openSet.append(neighbour)
    closest = closedSet[-1]
    for i in range(0,len(closedSet)):
        if closedSet[i].Node.eDist(graph2[goal].Node)<closest.Node.eDist(graph2[goal].Node):
            closest = closedSet[i]
    return reconstruct_path(closest)     

def findClosest(check):
    global graph
    closestIndex = 0
    dist = 0.1
    for i in range(len(graph)):
        ax=check.x
        ay=check.y
        az=check.z
        bx=graph[i].pose.position.x
        by=graph[i].pose.position.y
        bz=graph[i].pose.position.z
        nuDist = np.linalg.norm(np.array([ax,ay,az])-np.array([bx,by,bz]))
        if nuDist < dist:
            dist = nuDist
            closestIndex = i
    return closestIndex

def callback(data):
    si = findClosest(data.start_point)
    ei = findClosest(data.end_point)
    print(si, ei)
    A = A_Star(si,ei)
    if (type(A) is not str):
        g_limb.set_joint_position_speed(.1)
        a = datetime.datetime.now()
        for j in reversed(A):
            g_limb.move_to_joint_positions(j.angles, timeout=2)
        b = datetime.datetime.now()
        print(b-a)
        if True: ## CHANGE THIS TO FALSE IF YOU DON'T WANT THE NODE TO OUTPUT AN IMAGE OF THE WORKSPACE
            fig = plt.figure()
            for i in range(1,len(A)):
                Lx = np.array([A[i-1].pose.position.x,A[i].pose.position.x])
                Ly = np.array([A[i-1].pose.position.y,A[i].pose.position.y])
                Lz = np.array([A[i-1].pose.position.z,A[i].pose.position.z])
                plt.plot(Lx,Ly,c='r')
            for c in obstacles:
                y,x = c.exterior.xy
                plt.plot(x,y)
            plt.axis([0, 0.7, -0.7, 0.7])
            plt.show()
            fig.savefig('src/ahri_guidebot/plot.png')
        rospy.loginfo('Node is now accepting coordinates')

def main():
    RRT()
    rospy.loginfo('Node is now accepting coordinates')
    rospy.Subscriber("waypoint",Coordinates,callback)
    rospy.spin()

if __name__ == "__main__":
    main()
