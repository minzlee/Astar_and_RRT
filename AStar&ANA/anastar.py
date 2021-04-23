from Queue import PriorityQueue
from math import *
from numpy import *
import time

LARGENUM = 1E5

class DualPriorityQueue(PriorityQueue):
    def __init__(self, maxPQ=False):
        PriorityQueue.__init__(self)
        self.reverse = -1 if maxPQ else 1

    def put(self, priority, data):
        PriorityQueue.put(self, (self.reverse * priority, data))

    def get(self, *args, **kwargs):
        priority, data = PriorityQueue.get(self, *args, **kwargs)
        return self.reverse * priority, data

class Node:
    def __init__(self, x_in, y_in, theta_in):
        self.x = x_in
        self.y = y_in
        self.theta = theta_in
        self.g = LARGENUM     # cost from start node
        self.h = 0            # heuristic
        self.e = 0            # total cost
        self.parent = None

def drawPoints(env, result,color):
    h = []
    for (x, y, _) in result:
        h.append(env.plot3(points=array((x, y, 0.05)),pointsize=8.0,colors=array(color)))
    return h

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


def improveSol(openList, closedList, G, E, goalNode, env, robot, startT):
    x_step = 0.1
    y_step = 0.1
    angle_step = pi/4

    collisionList = []
    noncollisionList = []
    handles = []

    while not openList.empty():

        currNode = openList.get()[1]

        if currNode.e < E:
            E = currNode.e
        
        if sameNode(currNode, goalNode):
            G = currNode.g + currNode.h
            # here for the return values
            print("Suboptimal Solution Found!", G, 'Computation Time:', round(time.clock() - startT, 4))
            return currNode, openList, G, E, collisionList, noncollisionList

        neibConfig = []
        # 8-connected
        for i in [-x_step, 0.0, x_step]:
            for j in [-y_step, 0.0, y_step]:
                for ang in [-angle_step, 0.0, angle_step]:
                    if i == 0 and j == 0 and ang == 0:
                        continue
                    else:
                        neibConfig.append((currNode.x + i, currNode.y + j, currNode.theta + ang))

        
        for item in neibConfig:
            Flag = False
            nei = Node(item[0], item[1], angle_clip(item[2]))
            newGcost = currNode.g + cost(currNode, nei)
            nei.parent = currNode
            # check collision
            with env:
                robot.SetActiveDOFValues([nei.x, nei.y, nei.theta])
                if env.CheckCollision(robot):
                    collisionList.append((nei.x, nei.y, nei.theta))
                    continue
                else: 
                    noncollisionList.append((nei.x, nei.y, nei.theta))
                    if (nei.x, nei.y, nei.theta) not in closedList:
                        Flag = True
                    elif newGcost < closedList[(nei.x, nei.y, nei.theta)]:
                        Flag = True
            
            if Flag:
                closedList[(nei.x, nei.y, nei.theta)] = newGcost
                
                nei.g = newGcost
                nei.h = h(nei, goalNode)
                
                if nei.g + nei.h < G:
                    nei.e = round((G - nei.g) / nei.h, 5)
                    openList.put(nei.e, nei)
                

        # present the current closedList Node
        handles.append(env.plot3(points=array((currNode.x,currNode.y,0.05)),pointsize=8.0))
    return currNode, openList, G, E, collisionList, noncollisionList

def updateKeys(openList, G):
    newList = DualPriorityQueue(maxPQ=True)
    
    while openList.qsize():
        node = openList.get()[1]
        if node.g + node.h < G:
            node.e = (G - node.g) / node.h
            newList.put(node.e, node)
    print("new OPEN size: ", newList.qsize())
    return newList


def ANAStar(start, goal, robot, env, startT):
    # global startT
    # startT = time.clock()
    G = LARGENUM
    E = LARGENUM
    openList = DualPriorityQueue(maxPQ=True)

    # prepare startNode and goalNode
    startNode = Node(start[0], start[1],start[2])
    goalNode = Node(goal[0], goal[1],goal[2])
    # print("start: ", start[0], start[1],start[2])
    # print("goal: ", goal[0], goal[1],goal[2])

    startNode.g = 0
    startNode.h = h(startNode, goalNode)
    startNode.e = (G - startNode.g) / startNode.h
    goalNode.h = 1E-5

    # prepare openList and closedList and other List for the visualization
    openList.put(startNode.e, startNode)
    
    visted = {}
    visted[(start[0], start[1],start[2])] = startNode.g

    collisionShow = []
    noncollisionShow = []

    while openList.qsize():
        # step 1 improve the path
        finalNode, openList, G, E, collision, noncollision = improveSol(openList, visted, G, E, goalNode, env, robot, startT)
        
        print("G value is :", G)
        # step 2 get the grid
        collisionShow = collisionShow + collision
        noncollisionShow = noncollisionShow + noncollision

        # step 3 update the e(s) value for all node in OPEN
        openList = updateKeys(openList, G)
        # print(openList.qsize())

    # print("get here now! ")
    path = []
    nei = finalNode
    pathCost = G
    path.append((nei.x, nei.y, nei.theta))
    while nei is not startNode:
        path.append((nei.parent.x, nei.parent.y, nei.parent.theta))
        nei = nei.parent
                            
    path.reverse()
    # print("len path: ", len(path))

    return path, pathCost, collisionShow, noncollisionShow


