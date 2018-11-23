# -*- coding: utf-8 -*-
import time
from ePuckVRep import EPuckVRep
import numpy as np
import math
from docking import Docking
from searchDock import SearchDock
from approachDock import ApproachDock

from PIL import Image as I
resolX, resolY = 64, 64

def main():
    robot = EPuckVRep('ePuck', port=19999, synchronous=True)
    robot.startsim()
    robot.enablePose()
    robot.enableWheelEncoding()
    robot.enableCamera()

    image = I.new("RGB", (resolX, resolY), "white")

    #Goal position
    xGoal = 1.225
    yGoal = 0.0
    zetaGoal = 0
    goalPose = np.matrix([[xGoal], [yGoal], [zetaGoal]])

    #Control parameters
    kRoh = 0.6
    kAlpha = 6.5
    kBeta = -4   
    controlParameterMatrix = np.matrix([[kRoh][kAlpha][kBeta]])

    #subsumption behaviour list, order defines priorities!
    behaviours = [Docking(robot,goalPose,controlParameterMatrix), ApproachDock(resolX,resolY), SearchDock()] 

    # main sense-act cycle
    while robot.isConnected():
         stepCounter = 0

    # main sense-act cycle
    while robot.isConnected():
        stepCounter += 1
        if stepCounter%10 == 0 and (behaviour.getState() == 'searchDock' or behaviour.getState() == 'approachDock'):
            image = robot.getCameraImage()
            newImage = True
        else:
            newImage = False

        behaviour = next(behavior for behavior in behaviours if behavior.applicable(image, newImage))

        print("behaviour: ", behaviour.getState())

        leftMotor, rightMotor = behaviour.calculateMotorValues(image, newImage)

        robot.setMotorSpeeds(leftMotor, rightMotor)       
        #do one simulation step
        robot.stepsim(1)

    robot.disconnect()


if __name__ == '__main__':
    main()
