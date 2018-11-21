# -*- coding: utf-8 -*-
import time
from ePuckVRep import EPuckVRep
import numpy as np
import math


def main():
    robot = EPuckVRep('ePuck', port=19999, synchronous=False)
    robot.enablePose()
    robot.enableWheelEncoding()
    
    # xStart = 0.5
    # yStart = 0.5
    # zetaStart = 0
    # poseStart = np.matrix([[xStart],[yStart],[zetaStart]])

    xGoal = 1.225
    yGoal = 0.0
    zetaGoal = 0
    goalPose = np.matrix([[xGoal], [yGoal], [zetaGoal]])

    #Control parameters
    kRoh = 0.6
    kAlpha = 6.5
    kBeta = -4   
    sleepTime = 0.05

    r = robot._wheelDiameter/2
    l = robot._wheelDistance

    # Define the control matrix
    c00 = kRoh/r
    c01 = (kAlpha*l)/r
    c02 = (kBeta*l)/r
    c10 = kRoh/r
    c11 = -(kAlpha*l)/r
    c12 = -(kBeta*l)/r
    K = np.matrix([[c00,c01,c02],[c10,c11,c12]])

    reachedTarget = False

    temp = robot._getPose()
    xStart = temp[0]
    yStart = temp[1]
    zetaStart = temp[2]
    currentPose = np.matrix([[xStart],[yStart], [zetaStart]])
    print currentPose

    oldencval = robot.getWheelEncodingValues()
    #print oldencval

    # main sense-act cycle
    while robot.isConnected():
         #Get distance to goal
        deltaX = math.fabs(currentPose[0] - goalPose[0])
        deltaY = math.fabs(currentPose[1] - goalPose[1])
        deltaTheta = math.fabs(currentPose[2] - goalPose[2])

        if reachedTarget or (deltaX < 0.003  and deltaY < 0.002 and deltaTheta < 0.1) : 
            reachedTarget = True
            motorSpeed = np.matrix([[0],[0]])
        else:
            #Calculate the motor speeds so the robot moves torwards the goal
            polar = polarTransf(currentPose, goalPose)
            motorSpeed = K * polar

        robot.setMotorSpeeds(motorSpeed[1][0], motorSpeed[0][0])
        print '----- motor speed ----'
        print motorSpeed
        time.sleep(sleepTime)

        #Calculate the driven distance of the left and right wheel
        radius =robot._wheelDiameter/2
        encval = robot.getWheelEncodingValues()
        dsl, dsr, oldencval = calcDriven(encval, oldencval, radius)
        #Calculate the pose
        currentPose = calcCurrentPos(currentPose, robot._wheelDistance, dsr , dsl)
        #print (robot.getPose())
        #print (currentPose)

    robot.disconnect()


def calcDriven(encval, oldencval, radius):
    leftdist = (encval[0] + 2*math.pi - oldencval[0]) % (2*math.pi)
    rightdist = (encval[1] + 2*math.pi - oldencval[1]) % (2*math.pi)

    return leftdist, rightdist, encval

def calcCurrentPos(oldPos, wheelBase, drivenSr, drivenSl):
    deltaZeta = (drivenSr - drivenSl) / wheelBase
    deltaS = (drivenSr + drivenSl)/2
    c00 = deltaS*math.cos(oldPos[2] + deltaZeta/2)
    c10 = deltaS*math.sin(oldPos[2] + deltaZeta/2)
    c20 = deltaZeta
    temp = np.matrix([[c00], [c10], [c20]])
    print '-----temp------'
    print temp
    newPosition = oldPos + temp
    print '------new Pos-----'
    print newPosition
    return newPosition

def polarTransf(current, goal):
    deltaX = goal[0][0] - current[0]
    deltaY = goal[1][0] - current[1]
    roh = math.sqrt(deltaX**2 + deltaY**2)
    alpha = - current[2] + math.atan2(deltaY,deltaX)
    beta = - current[2] - alpha
    return np.matrix([[roh],[alpha],[beta]])


if __name__ == '__main__':
    main()
