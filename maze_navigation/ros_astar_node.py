#! /usr/bin/env python
import rospy
from nav_msgs.msg import OccupancyGrid
import numpy as np
import matplotlib.pyplot as plt
import time
from numba import jit
import math
from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
# from astar import AStar

pixwidth = 36.8
pixheight = 0

class MapMatrix:
    def __init__(self, map):
        self.w = map.shape[0]
        self.h = map.shape[1]
        self.data = map

    def showArrayD(self):
        for y in range(self.h):
            for x in range(self.w):
                print(self.data[x][y], end=' ')
            print("")

    def __getitem__(self, item):
        return self.data[item]

class Point:
    def __init__(self, x, y):
        self.x = x
        self.y = y

    def __eq__(self, other):
        if self.x == other.x and self.y == other.y:
            return True
        return False

class AStar:


    class Node:
        def __init__(self, point, endPoint, g=0):
            self.point = point  # point
            self.parent = None  # parent node
            self.g = g  # g value
            self.h = (abs(endPoint.x - point.x) + abs(endPoint.y - point.y))  # h value

    def __init__(self, map2d, startPoint, endPoint, passTag=0):
     
        # initialize the openset
        self.openList = []
        # initilize the close set
        self.closeList = []
        # set the map
        self.map2d = map2d
        # set the startPoint and endPoint
        if isinstance(startPoint, Point) and isinstance(endPoint, Point):
            self.startPoint = startPoint
            self.endPoint = endPoint
        else:
            self.startPoint = Point(*startPoint)
            self.endPoint = Point(*endPoint)

        # value of free space
        self.passTag = passTag

    def getMinNode(self):
 
        currentNode = self.openList[0]
        for node in self.openList:
            if node.g + node.h < currentNode.g + currentNode.h:
                currentNode = node
        return currentNode

    def pointInCloseList(self, point):
        for node in self.closeList:
            if node.point == point:
                return True
        return False

    def pointInOpenList(self, point):
        for node in self.openList:
            if node.point == point:
                return node
        return None

    def endPointInCloseList(self):
        for node in self.openList:
            if node.point == self.endPoint:
                return node
        return None

    def isValidPoint(self, x, y):
        if x < 0 or x > self.map2d.w - 1 or y < 0 or  y > self.map2d.h - 1:
            return False
        else:
            return True
        
    def nodeExpansion(self, minF, offsetX, offsetY):
       
        # if the point is not valid, skip it
        if not self.isValidPoint(minF.point.x + offsetX, minF.point.y + offsetY):
            return
        # if the point is occupied, skip it
        if self.map2d[minF.point.x + offsetX][minF.point.y + offsetY] != self.passTag:
            return
        # if the point is in the close set, skip it
        currentPoint = Point(minF.point.x + offsetX, minF.point.y + offsetY)
        if self.pointInCloseList(currentPoint):
            return
        # compute the step cost
        if offsetX == 0 or offsetY == 0:
            step = 1.0
        else:
            step = math.sqrt(2)
        currentNode = self.pointInOpenList(currentPoint)
        # if not in the open set, add it into the open set
        if not currentNode:
            currentNode = AStar.Node(currentPoint, self.endPoint, g=minF.g + step)
            currentNode.parent = minF
            self.openList.append(currentNode)
            return
        # if in the open set, compute the cost and update the value
        if minF.g + step < currentNode.g:  # if the cost is smaller, update the value and its parent node
            currentNode.g = minF.g + step
            currentNode.parent = minF

    def start(self):
     

        print('start')
        if self.map2d[self.endPoint.x][self.endPoint.y] != self.passTag:
            print('wrong target point')
            print(self.map2d[self.endPoint.x][self.endPoint.y])
            return None
        if self.map2d[self.startPoint.x][self.startPoint.y] != self.passTag:
            print('wrong start point')
            print(self.map2d[self.startPoint.x][self.startPoint.y])
            return None


        startNode = AStar.Node(self.startPoint, self.endPoint)
        self.openList.append(startNode)


        while True:
            minF = self.getMinNode()
            self.closeList.append(minF)
            self.openList.remove(minF)
            self.nodeExpansion(minF, 0, -1)
            self.nodeExpansion(minF, 0, 1)
            self.nodeExpansion(minF, -1, 0)
            self.nodeExpansion(minF, 1, 0)
            point = self.endPointInCloseList()
            if point:  
                cPoint = point
                pathList = []
                while True:
                    if cPoint.parent:
                        # pathList.append(cPoint.point)
                        pathList.append([cPoint.point.y, cPoint.point.x])
                        cPoint = cPoint.parent
                    else:
                        # print(pathList)
                        # print(list(reversed(pathList)))
                        # print(pathList.reverse())
                        return list(reversed(pathList))
            if len(self.openList) == 0:
                return None



@jit(nopython=True)
def _obstacleMap(map, obsize):
    '''
    obsize: the inflated index
    '''
    inflation_cells = []
    height = map.shape[0]
    width = map.shape[1]
    Squobsize = obsize * obsize
    for offset_x in range(-obsize, obsize + 1):
        for offset_y in range(-obsize, obsize + 1):
            disSqu = offset_x * offset_x + offset_y * offset_y
            if disSqu > 0 and disSqu < Squobsize:
                inflation_cells.append([offset_x,offset_y])
    indexList = np.where(map == 1)
    for ox, oy in zip(indexList[0], indexList[1]):
        for i in range(len(inflation_cells)):
            offset_x = inflation_cells[i][0]
            offset_y = inflation_cells[i][1]
            x = ox + offset_x
            y = oy + offset_y
            if x>=0 and x<height  and y >=0 and y<width:
                map[x][y]=1
        # distance = math.sqrt((x - ox) ** 2 + (y - oy) ** 2)
        # if distance <= obsize:
        #     map[x][y] = 1
    # # terrible implementation! wait for rewriting
    # for x in range(map.shape[0]):
    #     for y in range(map.shape[1]):
    #         if map[x][y] == 0:



class pathPlanning():
    def __init__(self):

        rospy.init_node("Astar_path_planning", anonymous=True)
        self.Map_Callback()
        self.robot_radius = 0.4
        print("Inflating the obstacle map")
        ob_time = time.time()
        self.obsize = math.ceil(self.robot_radius / self.resolution)
        _obstacleMap(self.map, self.obsize)
        print("The time of inflation is :{:.3f}".format(time.time() - ob_time))

        self.map2d = MapMatrix(self.map)
        self.start_point= [0,7]
        self.target_point = [366,360]
        astar_time = time.time()
        aStar = AStar(self.map2d, Point(self.start_point[1], self.start_point[0]),
                      Point(self.target_point[1], self.target_point[0]))
        self.pathList = aStar.start()
        print("The time of path planning is :{:.3f}".format(time.time() - astar_time))
        self.sendAstarPath()

    def Map_Callback(self):
        self.OGmap = rospy.wait_for_message("/map", OccupancyGrid, timeout=None)
        self.width = self.OGmap.info.width
        self.height = self.OGmap.info.height
        self.resolution = self.OGmap.info.resolution

        # get the data from OGmap. Occupied space is 100, free space is 0, and unknown is -1
        mapdata = np.array(self.OGmap.data, dtype=np.int8)
        # transforming the data into a matrix
        self.map = mapdata.reshape((self.height, self.width))

        # set values of occupied elements to 1
        self.map[self.map == 100] = 1
        # reversing the matrix
        self.map = self.map[:, ::-1]


    def sendAstarPath(self):
        AstarPath = rospy.Publisher("AstarPath", Path, queue_size=15)
        init_path = Path()

        # set the rate of publisher
        rate = rospy.Rate(200)

        for i in range(len(self.pathList)):
            init_path.header.stamp = rospy.Time.now()
            init_path.header.frame_id = "map"

            current_point = PoseStamped()
            current_point.header.frame_id = "map"
            current_point.pose.position.x = pixwidth - self.pathList[i][0] * self.resolution
            current_point.pose.position.y = self.pathList[i][1] * self.resolution - pixheight
            current_point.pose.position.z = 0
            # set the orientation
            current_point.pose.orientation.x = 0
            current_point.pose.orientation.y = 0
            current_point.pose.orientation.z = 0
            current_point.pose.orientation.w = 1

            init_path.poses.append(current_point)
            # publish the path
            AstarPath.publish(init_path)

            rate.sleep()
            i += 1
        time.sleep(0.5)

    def worldToMap(self, x, y):
        mx = (int)((pixwidth - x) / self.resolution)
        my = (int)(-(-pixheight - y) / self.resolution)
        return [mx, my]


if __name__ == "__main__":
    getmap = pathPlanning()





