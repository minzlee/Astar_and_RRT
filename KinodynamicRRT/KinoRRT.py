import random
import time
from math import *
from numpy import *

L = 0.65
A = L ** 2
Threshold = 0.5
K = 5
MaxSpeed = 2

class Node:
    def __init__(self, x, y, theta, xVel=0, yVel=0, thetaVel=0):
        self.x = x
        self.y = y
        self.theta = theta
        self.xVel = xVel
        self.yVel = yVel
        self.thetaVel = thetaVel
        self.parent = None

class Action:
    def __init__(self, vel, phi):
        self.vel = vel
        self.phi = phi


def dist(c1, c2):
    dist4ThetaSq = A * min(pow(c2.theta - c1.theta, 2), pow(c2.theta - c1.theta + 2*pi, 2), pow(c2.theta - c1.theta - 2*pi, 2))
    return sqrt(pow(c2.x - c1.x, 2) + pow(c2.y - c1.y, 2) + dist4ThetaSq)

def findNearestNode(tree, randNode):
    nearestNode = tree[0]
    distMin = dist(nearestNode, randNode)
    for node in tree:
        distnew = dist(node, randNode)
        if distnew < distMin:
            nearestNode = node
            distMin = distnew
    return nearestNode, distMin

def dist4Goal(c1, c2):
    distxy = sqrt(pow(c2.x - c1.x, 2) + pow(c2.y - c1.y, 2))
    distAng = abs(c2.theta - c1.theta)

    while distAng > 2*pi:
        distAng -= 2*pi

    if distxy < Threshold and abs(distAng) < pi/3:
        return True
    
    return False

def reachGoalArea(tree, goalNode):
    for node in tree:
        if dist4Goal(node, goalNode):
            return True, node
    
    return False, None


def genPrimitives(act, nearestNode, env, robot):
    notValid = 0
    Primitive = []
    dt =  0.2
    extendNode = nearestNode
    velo = sqrt(extendNode.xVel**2 + extendNode.yVel**2)

    for j in range(6):
        velo += act.vel * dt

        if velo > MaxSpeed:
            velo = MaxSpeed
        if velo < -MaxSpeed:
            velo = -MaxSpeed
        phi = tan(act.phi) * (velo / L)

        newTheta = extendNode.theta + dt * phi
        newNode = Node(extendNode.x + cos(extendNode.theta) * dt * velo,
                   extendNode.y + sin(extendNode.theta) * dt * velo, 
                   newTheta,
                   cos(extendNode.theta) * velo,
                   sin(extendNode.theta) * velo,
                   phi)

        # check if the node is inside the env
        if (newNode.x - (-10.0)) * (newNode.x - 10.0) > 0 or (newNode.y - (-10.0)) * (newNode.y - 10.0) > 0:
            notValid = 1
            break
                    
        with env:
            robot.SetTransform([[cos(newNode.theta), -sin(newNode.theta), 0., newNode.x],
                            [sin(newNode.theta), cos(newNode.theta), 0., newNode.y],
                            [0., 0., 1., 0.8],
                            [0., 0., 0., 1.]])
                                        
            if env.CheckCollision(robot):
                notValid = 1
                break

        newNode.parent = extendNode
        Primitive.append(newNode)
        extendNode = newNode
    return Primitive, notValid     


def genPath(goalNode, startNode):
    path = []
    current = goalNode
    while current is not startNode:
        path.append((current.x, current.y, current.theta))
        current = current.parent
    path.append((startNode.x, startNode.y, startNode.theta))
    path.reverse()
    return path

def randomGen(goalNode):
    if random.randint(1, 5) == 1:
        return goalNode
    
    randomConfig = []
    randomConfig.append(random.uniform(-9.5, 9.5)) # get the x range in env
    randomConfig.append(random.uniform(-9.5, 9.5)) # get the y range in env
    randomConfig.append(random.uniform(-pi, pi))
        
    return Node(randomConfig[0], randomConfig[1], randomConfig[2])

def RRT(start, goal, env, robot, actionListChoice=0):
    startTime = time.clock()
    tree = []
    startNode = Node(start[0], start[1], start[2])
    goalNode = Node(goal[0], goal[1], goal[2])
    tree.append(startNode)
    samplePoints = []
    primitives = []

    print "\n\ndistance from start to goal", dist(startNode, goalNode)
    raw_input("Press enter to exit...")
    print "\n\nstart to sampling and find the path\n******May take few mintus****"

    while True:
        # check if we reach the goal area
        flag, finalNode = reachGoalArea(tree, goalNode)
        if flag:
            break

        # generate random point and check if it got collison
        randomNode = randomGen(goalNode)
        # print "randomNode", randomNode.x, "  ", randomNode.y
        with env:
            robot.SetTransform([[cos(randomNode.theta),  -sin(randomNode.theta),  0.,  randomNode.x],
                            [sin(randomNode.theta),  cos(randomNode.theta),  0.,  randomNode.y],
                            [0.,  0.,  1.,   0.8],
                            [0.,  0.,  0.,   1.]])
            if env.CheckCollision(robot):
                continue
        samplePoints.append(env.plot3(points=array((randomNode.x, randomNode.y, 0.1)), pointsize=5.0,
                           colors=array((0, 0, 0))))

        # find nearest node 
        nearestNode, distMin = findNearestNode(tree, randomNode)

        # expand process
        if distMin <= Threshold:
            continue
        else:
            # print "here in the expand proecess"
            actionList = [
                          Action(0.5, pi/6), Action(0.5, 0), Action(0.5, -pi/6),
                          Action(-0.5, pi/6), Action(-0.5, 0), Action(-0.5, -pi/6),
                          Action(0.8, pi/8), Action(0.8, 0), Action(0.8, -pi/8),
                          Action(-0.8, pi/8), Action(-0.8, 0), Action(-0.8, -pi/8)
                          ]

            if actionListChoice == 3:
                actionList = [
                             Action(0.8, pi/8), Action(0.8, 0), Action(0.8, -pi/8),
                             Action(-0.8, 0)
                             ]
            elif actionListChoice == 2:
                actionList = [
                          Action(0.8, pi/8), Action(0.8, 0), Action(0.8, -pi/8),
                          Action(-0.8, pi/8), Action(-0.8, 0), Action(-0.8, -pi/8)
                          ]
            elif actionListChoice == 1:
                actionList = [
                          Action(0.5, pi/6), Action(0.5, 0), Action(0.5, -pi/6),
                          Action(-0.5, pi/6), Action(-0.5, 0), Action(-0.5, -pi/6),
                          Action(0.8, pi/8), Action(0.8, 0), Action(0.8, -pi/8),
                          Action(-0.8, pi/8), Action(-0.8, 0), Action(-0.8, -pi/8),
                          Action(0.25, pi/6), Action(0.25, 0), Action(0.25, -pi/6),
                          Action(-0.25, pi/6), Action(-0.25, 0), Action(-0.25, -pi/6),
                          Action(0.4, pi/10), Action(0.4, 0), Action(0.4, -pi/10),
                          Action(-0.4, pi/10), Action(-0.4, 0), Action(-0.4, -pi/10),
                          Action(0.25, pi/3), Action(0.25, 0), Action(0.25, -pi/3),
                          Action(-0.25, pi/3), Action(-0.25, 0), Action(-0.25, -pi/3)
                          ]

            extendNode = nearestNode
            distMinPrev = float('inf')
            num = 0
            # print "get hrere"
            while num < K:
                distMin = float('inf')
                
                for act in actionList:                   
                    notValid = 0
                    onePrimitive, notValid = genPrimitives(act, nearestNode, env, robot)

                    if notValid:
                        continue
                    

                    for idx in range(1, len(onePrimitive)):
                        # print "getin here"
                        tree.append(onePrimitive[idx])
                        primitives.append(env.drawlinestrip(points=array(((onePrimitive[idx].x, onePrimitive[idx].y, 1),
                                                                (onePrimitive[idx-1].x, onePrimitive[idx-1].y, 1))),
                                                                linewidth=2.0, colors=array((1, 0, 0))))

                    samplePoints.append(env.plot3(points=array((onePrimitive[-1].x, onePrimitive[-1].y, 1)), pointsize=4.0,
                                       colors=array((1, 1, 0))))
                    # raw_input("Press enter to exit...")

                    currDist = dist(onePrimitive[-1], randomNode)
                    # print " currDist ", currDist

                    if currDist < distMin:
                        distMin = currDist
                        bestNode = onePrimitive[-1]
                
                # raw_input("Press enter to exit...")
                if distMin <= 1.5 * Threshold or distMinPrev - distMin <= 1.5 * Threshold:
                    break

                distMinPrev = distMin
                nearestNode = bestNode
                num += 1
    
    # endTime = time.clock()
    # print "Time used to find path to the goal region:", endTime - startTime, '\n'
    # raw_input("Press enter to exit...")
    endTime = time.clock()
    print "\nTime used to find path to the goal region:", endTime - startTime, 's\n'
    path = genPath(finalNode, startNode)

    raw_input("Press enter to continue...")
    return path