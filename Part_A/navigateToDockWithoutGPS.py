# -*- coding: utf-8 -*-
import time
from ePuckVRep import EPuckVRep
import numpy as np
import math


def main():
    robot = EPuckVRep('ePuck', port=19999, synchronous=False)

    # xStart = 0.5
    # yStart = 0.5
    # zetaStart = 0
    # poseStart = np.matrix([[xStart],[yStart],[zetaStart]])

    xGoal = 1.225
    yGoal = 0.0
    zetaGoal = 0
    goalPose = np.matrix([[xGoal], [yGoal], [zetaGoal]])

    # Control parameters
    kRoh = 1
    kAlpha = 1
    kBeta = 1
    r = 0.0425  # in meters
    l = 0.0541  # in meters

    # Define the control matrix
    c00 = kRoh / r
    c01 = (kAlpha * l) / r
    c02 = (kBeta * l) / r
    c10 = kRoh / r
    c11 = -(kAlpha * l) / r
    c12 = -(kBeta * l) / r
    K = np.matrix([[c00, c01, c02], [c10, c11, 12]])
    currentPose = robot._getPose()

    # main sense-act cycle
    while robot.isConnected():
        motorSpeed = K * polarTransf(currentPose, goalPose)

        robot.setMotorSpeeds(motorSpeed[0][0], motorSpeed[1][0])

        time.sleep(0.05)
        #TODO prüfen ob die Werte der distanz, der summe der disanz oder etwas anderem entsprechen siehe skriptum merz s20
        driven = robot.getWheelEncodingValues()
        currentPose = calcCurrentPos(currentPose, robot._wheelDistance, driven[1], driven[2])

    robot.disconnect()

def calcCurrentPos(oldPos, wheelBase, drivenSr, drivenSl):
    c00 = (drivenSr + drivenSl)/2*math.cos(oldPos[2][0] + ((drivenSr-drivenSl)/(2*wheelBase)))
    c10 = (drivenSr + drivenSl)/2*math.sin(oldPos[2][0] + ((drivenSr-drivenSl)/(2*wheelBase)))
    c20 = (drivenSr-drivenSl)/wheelBase
    temp = np.matrix([[c00], [c10], [c20]])
    return oldPos + temp

def polarTransf(currentPose, goalPose):
    deltaX = goalPose[0][0] - currentPose[0][0]
    deltaY = goalPose[1][0] - currentPose[1][0]
    roh = math.sqrt(deltaX ** 2 + deltaY ** 2)
    alpha = - currentPose[2][0] + math.atan2(deltaY, deltaX)
    beta = - currentPose[2][0] - alpha
    return np.matrix([[roh], [alpha], [beta]])


if __name__ == '__main__':
    main()
