# -*- coding: utf-8 -*-
import time
from ePuckVRep import EPuckVRep
import numpy as np
import math

def main():
    robot = EPuckVRep('ePuck',port=19999, synchronous=False)
    robot.enablePose()

    #xStart = 0.5
    #yStart = 0.5
    #zetaStart = 0
    #poseStart = np.matrix([[xStart],[yStart],[zetaStart]])

    xGoal = 1.225
    yGoal = 0.0
    zetaGoal = 0
    goalPose = np.matrix([[xGoal],[yGoal],[zetaGoal]])

    #Control parameters
    kRoh = 0.5
    kAlpha = 6.5
    kBeta = -4

    r = robot._wheelDiameter/2
    l = robot._wheelDistance

    #Define the control matrix
    c00 = kRoh/r
    c01 = (kAlpha*l)/r
    c02 = (kBeta*l)/r
    c10 = kRoh/r
    c11 = -(kAlpha*l)/r
    c12 = -(kBeta*l)/r
    K = np.matrix([[c00,c01,c02],[c10,c11,c12]])

    reachedTarget = False

    # main sense-act cycle
    while robot.isConnected():
        
        currentPose = robot._getPose()

        deltaX = math.fabs(currentPose[0] - goalPose[0])
        deltaY = math.fabs(currentPose[1] - goalPose[1])
        deltaTheta = math.fabs(currentPose[2] - goalPose[2])

        print currentPose
        print deltaX
        print deltaY
        print deltaTheta
        print reachedTarget
        print '----------------------'

        if reachedTarget or (deltaX < 0.003  and deltaY < 0.002 and deltaTheta < 0.1) : 
            reachedTarget = True
            motorSpeed = np.matrix([[0],[0]])
        else:
            polar = polarTransf(currentPose, goalPose)

            motorSpeed = K * polar

        robot.setMotorSpeeds(motorSpeed[1][0], motorSpeed[0][0])

        time.sleep(0.05)

    robot.disconnect()

def polarTransf(current, goal):
    deltaX = goal[0][0] - current[0]
    deltaY = goal[1][0] - current[1]
    roh = math.sqrt(deltaX**2 + deltaY**2)
    alpha = - current[2] + math.atan2(deltaY,deltaX)
    beta = - current[2] - alpha
    return np.matrix([[roh],[alpha],[beta]])

if __name__ == '__main__':
    main()