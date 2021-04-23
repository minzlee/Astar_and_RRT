from Queue import PriorityQueue
from math import *
from numpy import *

EPS = 1E-3

class Node:
    def __init__(self, x_in, y_in, theta_in):
        self.x = x_in
        self.y = y_in
        self.theta = theta_in
        self.g = 0   # cost from start node
        self.h = 0   # heuristic
        self.f = 0   # total cost
        self.parent = None


def angle_clip(angle):
    while angle <= -pi or angle > pi:
        if angle <= -pi:
            angle += 2*pi
        if angle > pi:
            angle -= 2*pi
    return angle

def cost(n, m):
    # action cost
    return sqrt(pow(n.x - m.x, 2)+pow(n.y - m.y, 2)+pow(min(abs(n.theta - m.theta), 2*pi - abs(n.theta - m.theta)), 2))

def h(n, g, hchoice='Euclidean'):
    # Manhattan distance heuristic
    if hchoice == 'Manhattan':
        return abs(n.x - g.x) + abs(n.y - g.y) + min(abs(n.theta - g.theta), 2*pi - abs(n.theta - g.theta)) # could add threshold for the theta
    # Euclidean distance heuristic
    else:
        return cost(n, g)

def sameNode(n, m):
    return sqrt((n.x - m.x)**2 + (n.y - m.y)**2) < 0.1 and abs(n.theta - m.theta) < pi/4


def AStar(start, goal, robot, env, connected, distance_type):
    startNode = Node(start[0], start[1],start[2])
    goalNode = Node(goal[0], goal[1],goal[2])

    openList = PriorityQueue()
    closeSet = set()
    openSet = set()

    collisionList = []
    openList.put((0, startNode))
    openSet.add((startNode.x, startNode.y, startNode.theta))
    x_step = 0.1
    y_step = 0.1
    
    path = []
    pathCost = 0.0
    handles = []

    while not openList.empty():
        currNode = openList.get()[1]        
        
        neibConfig = []
        if connected == 4:
            # four connected
            angle_step = pi/2
            neibConfig = [  (currNode.x + x_step, currNode.y ,         currNode.theta),
                        (currNode.x - x_step, currNode.y ,         currNode.theta),
                        (currNode.x,          currNode.y + y_step, currNode.theta),
                        (currNode.x,          currNode.y - y_step, currNode.theta),
                        (currNode.x,          currNode.y ,         currNode.theta + angle_step),
                        (currNode.x,          currNode.y ,         currNode.theta - angle_step)
                        ]
        else:
            # 8-connected
            angle_step = pi/4
            for i in [-x_step, 0.0, x_step]:
                for j in [-y_step, 0.0, y_step]:
                    for ang in [-angle_step, 0.0, angle_step]:
                        if i == 0 and j == 0 and ang == 0:
                                continue
                        else:
                            neibConfig.append((currNode.x + i, currNode.y + j, currNode.theta + ang))
        
        for item in neibConfig:
            nei = Node(item[0], item[1], angle_clip(item[2]))
            nei.parent = currNode

            nei.g = currNode.g + cost(currNode, nei)
            nei.h = h(nei, goalNode, distance_type)
            nei.f = nei.g + nei.h
            
            # check if we reach the goalNode
            if sameNode(goalNode, nei):
                pathCost = nei.f
                path.append((nei.x, nei.y, nei.theta))
                
                while nei is not startNode:
                    path.append((nei.parent.x, nei.parent.y, nei.parent.theta))
                    nei = nei.parent
                            
                path.reverse()

                return path, pathCost, closeSet, collisionList
            
            # check collision
            collision_check = False
            with env:
                robot.SetActiveDOFValues([nei.x, nei.y, nei.theta])
                collision_check = env.CheckCollision(robot)
                if env.CheckCollision(robot):
                    collisionList.append((nei.x, nei.y, nei.theta))
                    continue
            
            if (nei.x, nei.y, nei.theta) not in closeSet and collision_check == False:
                if (nei.x, nei.y, nei.theta) not in openSet:
                    openList.put((nei.f, nei))
                    openSet.add((nei.x, nei.y, nei.theta))
                    

        closeSet.add((currNode.x,currNode.y, currNode.theta))
        # print(len(closeSet))
        # print the current closedList nodes
        handles.append(env.plot3(points=array((currNode.x,currNode.y,0.05)),pointsize=8.0,colors=array((0,0,1))))

    return path, pathCost, closeSet, collisionList