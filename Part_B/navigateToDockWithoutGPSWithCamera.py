# -*- coding: utf-8 -*-
import time
from ePuckVRep import EPuckVRep
import numpy as np
import math
from docking import Docking
from searchDock import SearchDock
from approachDock import ApproachDock

from PIL import Image as I 

def calcDriven(encval, oldencval, radius):
    leftdist = (encval[0] + 2*math.pi - oldencval[0]) % (2*math.pi) * radius
    rightdist = (encval[1] + 2*math.pi - oldencval[1]) % (2*math.pi) * radius

    return leftdist, rightdist

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

def updatePose(currentPose, oldencval, wheelRadius, wheelDistance, encval):
    #Calculate the driven distance of the left and right wheel
    dsl, dsr = calcDriven(encval, oldencval, wheelRadius)
    print dsl, dsr
    #Calculate the pose
    pose = calcCurrentPos(currentPose, wheelDistance, dsr , dsl)
    return pose, encval
    
def getInitialPose(robot):
    # get start Position and convert to numpy-matrix
    temp = robot._getPose()
    xStart = temp[0]
    yStart = temp[1]
    zetaStart = temp[2]
    return np.matrix([[xStart],[yStart], [zetaStart]])

def main():
    stepCounter = 0

    robot = EPuckVRep('ePuck', port=19999, synchronous=True)
    robot.startsim()
    robot.enablePose()
    robot.enableWheelEncoding()
    robot.enableCamera()

    resolX, resolY = 64, 64
    image = I.new("RGB", (resolX, resolY), "white")

    wheelRadius = robot._wheelDiameter/2
    wheelDistance = robot._wheelDistance

    #Goal position
    xGoal = 1.225
    yGoal = 0.0
    zetaGoal = 0
    goalPose = np.matrix([[xGoal], [yGoal], [zetaGoal]])

    #Control parameters
    #kRoh = 0.6
    #kAlpha = 6.5
    #kBeta = -4 
    kRoh = 0.4
    kAlpha = 1.2
    kBeta = -0.02
    #controlParameterMatrix = np.matrix([[kRoh][kAlpha][kBeta]])

    #subsumption behaviour list, order defines priorities!
    behaviours = [Docking(wheelRadius,wheelDistance, resolX, resolY), ApproachDock(resolX,resolY), SearchDock()] 

    currentPose = getInitialPose(robot)
    oldencval = robot.getWheelEncodingValues()
    print oldencval

    # main sense-act cycle
    while robot.isConnected():
        stepCounter += 1
        if stepCounter%10 == 0 and (behaviour.getState() == 'searchDock' or behaviour.getState() == 'approachDock'):
            image = robot.getCameraImage()
            newImage = True
        else:
            newImage = False

        behaviour = next(behavior for behavior in behaviours if behavior.applicable(image, newImage, currentPose, goalPose, kRoh, kAlpha, kBeta))

        print("behaviour: ", behaviour.getState())
        if behaviour.getState() == 'Docking':
            print 'docking'
        leftMotor, rightMotor = behaviour.calculateMotorValues(image, newImage, currentPose, goalPose, kRoh, kAlpha, kBeta)

        robot.setMotorSpeeds(leftMotor, rightMotor)       
        #do one simulation step
        robot.stepsim(1)

        encval = robot.getWheelEncodingValues()
        print encval
        currentPose, oldencval = updatePose(currentPose, oldencval, wheelRadius, wheelDistance, encval)
        print'----- real Pose -----'
        print robot.getPose()

    robot.disconnect()

if __name__ == '__main__':
    main()