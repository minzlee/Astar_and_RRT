#!/usr/bin/env python
# -*- coding: utf-8 -*-
import time
import openravepy
from KinoRRT import *

if not __openravepy_build_doc__:
    from openravepy import *
    from numpy import *

def waitrobot(robot):
    """busy wait for robot completion"""
    while not robot.GetController().IsDone():
        time.sleep(0.01)

def drawPoints(result,color):
    h = []
    for item in result:
        xy = [item[0], item[1]]
        h.append(env.plot3(points=array((xy[0],xy[1],0.05)),pointsize=8.0,colors=array(color)))
    return h

def ConvertPathToTrajectory(robot,path=[]):
    #Path should be of the form path = [q_1, q_2, q_3,...], where q_i = [x_i, y_i, theta_i]
    if not path:
        return None
    # Initialize trajectory
    traj = RaveCreateTrajectory(env,'')    
    traj.Init(robot.GetActiveConfigurationSpecification())
    for i in range(0,len(path)):
        traj.Insert(i,numpy.array(path[i]))
    # Move Robot Through Trajectory
    planningutils.RetimeAffineTrajectory(traj,maxvelocities=ones(3),maxaccelerations=5*ones(3))
    return traj

if __name__ == "__main__":
    env = Environment()
    env.SetViewer('qtcoin')
    collisionChecker = RaveCreateCollisionChecker(env,'ode')
    env.SetCollisionChecker(collisionChecker)

    
    env.Reset()
    # load a scene from ProjectRoom environment XML file
    env.Load('my.env.xml')
    time.sleep(0.1)

    # 1) get the 1st robot that is inside the loaded scene
    # 2) assign it to the variable named 'robot'
    robot = env.GetRobots()[0]

    # set the DOF for my robot
    robot.SetActiveDOFs([],DOFAffine.X|DOFAffine.Y|DOFAffine.RotationAxis,[0,0,1])

    # set the start postion and the goal postion:
    start = [9, 9, pi]
    # start = [8, 3, pi]
    goal = [-7, -4.5, pi/4]
    actionListChoice = 0

    # show in the env
    gaolShow = drawPoints([goal], array((1, 0, 0)))
    handles = []
    handles.append(env.drawlinestrip(points=array(((-6.3,-5.2,0.05),(-7.7,-5.2,0.05),(-7.7,-3.7,0.05), (-6.3,-3.7,0.05), (-6.3,-5.2,0.05))),
                                           linewidth=3.0,
                                           colors=array(((0,1,0),(0,1,0),(0,1,0),(0,1,0),(0,1,0)))))

    # get the path
    with env:
        h = []
        path = RRT(start, goal, env, robot, actionListChoice)
        raw_input("Press enter to draw the path...")
        pathShow = drawPoints(path, array((0, 0, 0)))
    
    traj = ConvertPathToTrajectory(robot, path)
    if traj != None:
        raw_input("Press enter to excute the robot...")
        robot.GetController().SetPath(traj)
    
    waitrobot(robot)

    raw_input("Press enter to exit...")
    env.Destroy()
