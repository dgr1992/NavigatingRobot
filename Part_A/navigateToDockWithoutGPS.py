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

    # Control parameters
    kRoh = 0.5
    kAlpha = 5
    kBeta = -3.5
    r = 0.0425  # in meters
    l = 0.0541  # in meters

    # Define the control matrix
    c00 = kRoh/r
    c01 = (kAlpha*l)/r
    c02 = (kBeta*l)/r
    c10 = kRoh/r
    c11 = -(kAlpha*l)/r
    c12 = -(kBeta*l)/r
    K = np.matrix([[c00,c01,c02],[c10,c11,c12]])


    temp = robot._getPose()
    xStart = temp[0]
    yStart = temp[1]
    zetaStart = temp[2]
    currentPose = np.matrix([[xStart],[yStart], [zetaStart]])

    oldencval = robot.getWheelEncodingValues()

    # main sense-act cycle
    while robot.isConnected():
        pol = polarTransf(currentPose, goalPose)

        motorSpeed = K * pol

        robot.setMotorSpeeds(motorSpeed[0][0], motorSpeed[1][0])

        time.sleep(0.05)
        #TODO prüfen ob die Werte der distanz, der summe der disanz oder etwas anderem entsprechen siehe skriptum merz s20

        radius =robot._wheelDiameter/2

        dsl, dsr, oldencval = calcDriven(robot, oldencval, radius)
        print dsl
        print dsr

        currentPose = calcCurrentPos(currentPose, robot._wheelDistance, dsr , dsl)

        # print currentPose
        # print '----------------------'

    robot.disconnect()


def calcDriven(robot, oldencval, rad):
    encval = robot.getWheelEncodingValues()
    # TODO FIX distances mod2pi
    leftdist = (oldencval[0] - encval[0]) * rad
    rightdist = (oldencval[1] - encval[1]) * rad
    oldencval = encval
    return leftdist, rightdist, oldencval

def calcCurrentPos(oldPos, wheelBase, drivenSr, drivenSl):
    c00 = (drivenSr + drivenSl)/2*math.cos(oldPos[2] + ((drivenSr-drivenSl)/(2*wheelBase)))
    c10 = (drivenSr + drivenSl)/2*math.sin(oldPos[2] + ((drivenSr-drivenSl)/(2*wheelBase)))
    c20 = (drivenSr-drivenSl)/wheelBase
    temp = np.matrix([[c00], [c10], [c20]])
    return oldPos + temp

def polarTransf(currentPos, goalPos):
    deltaX = goalPos.item(0) - currentPos.item(0)
    deltaY = goalPos.item(1) - currentPos.item(1)
    temp = deltaX**2 + deltaY**2
    roh = math.sqrt(temp)
    alpha = - currentPos[2] + math.atan2(deltaY,deltaX)
    beta = - currentPos[2] - alpha
    return np.matrix([[roh],[alpha],[beta]])


if __name__ == '__main__':
    main()
