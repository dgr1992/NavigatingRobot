# -*- coding: utf-8 -*-
"""
Created on Wed Oct 31 16:40:22 2017
subsumption solution for exercise 3b
make sure to start first VRep scene boxPushingExercise3.ttt, then this program


@author: hans vollbrecht
"""
import time
from ePuckVRep import EPuckVRep
import numpy as np
from PIL import Image as I

resolX, resolY = 64, 64



#in the following, the subsumption behaviour classes:
from abc import ABCMeta, abstractmethod

class Behaviour:
    # the abstract superclass of all subsumption behaviours
    __metaclass__ = ABCMeta

    def __init__(self, state):
        """
        :param state:  string
            the name of this behaviour
        """
        self._maxVel = 120 * np.pi / 180  # 4/3 of a full wheel turn
        self.__state = state


    def getState(self):
        return self.__state


    def normalizeDistances(self, distances, noDetectionDistance):
        """
        ePuck has higher proximity values for lower distances; also normalize to [0,1]
        :param distances:  numpy float array
            distances measured by proximity sensors; in meters
        :param noDetectionDistance:  float
            maximum distance in meters, same property for all proximity sensors: 0.05 for ePuck
        :return: distances: numpy float array
            normalized distances measured by proximity sensors
        """
        distances = 1 - (distances / noDetectionDistance)
        return distances

    @abstractmethod
    def applicable(self, distances, noDetectionDistance, accel, image, stepCounter, newImage):
        """
        :param distances: numpy float array
            distances measured by proximity sensors; in meters
            starts with index 0: far left, in clockwise sequence
        :param noDetectionDistance: float
            maximum distance in meters, same property for all proximity sensors: 0.05 for ePuck
        :return:  boolean
            behaviour is applicable in current situation
        """
        pass

    @abstractmethod
    def calculateMotorValues(self, distances, noDetectionDistance, accel, image, stepCounter, newImage):
        """
        :param distances: numpy float array
            distances measured by proximity sensors; in meters
            starts with index 0: far left, in clockwise sequence
        :param noDetectionDistance: float
            maximum distance in meters, same property for all proximity sensors: 0.05 for ePuck
        :return:  (float,float)
            left and right motor velocity
        """
        pass



class SearchBox(Behaviour):

    def __init__(self):
        super(SearchBox, self).__init__("searchBox")

    def applicable(self, distances, noDetectionDistance, accel, image, stepCounter, newImage):
        """
        :param see method in Behaviour class
        :return:  boolean
            behaviour is applicable in current situation
        """
        return True


    def calculateMotorValues(self, distances, noDetectionDistance, accel, image, stepCounter, newImage):
        """
        :param see method in Behaviour class
        :return:  (float,float)
            left and right motor velocity
        """
        return 0.1, -0.1    # turn clockwise, since box detection in image searches from left to right,
                            # and we want to detect the same box again, not a second one appearing now



class ApproachBox(Behaviour):

    def __init__(self, resolutionX, resolutionY):
        self._xCenter = [-1]
        self._resolX = resolutionX
        self._resolY = resolutionY
        super(ApproachBox, self).__init__("approachBox")

    def detectBox(self, image):
        """
        looks in current image for a black blob on a red background, from left to right
        :param
                image: PIL.Image
                    a rgb image with black blobs on red background

        :return: true,  if black blob found.
                side-effect: if blob found, the function calculates xCenter: [int]
                    the center of the image: result of the function

        """
        minBlobWidth = 5
        xStart = -1
        for y in range(self._resolY):
            blobwidth = 0
            for x in range(self._resolX):
                pixel = image.getpixel((x, y))
                if pixel == (0, 0, 0):  # black pixel: a box!
                    blobwidth += 1
                    if blobwidth == 1:
                        xStart = x
                else:
                    if blobwidth >= minBlobWidth:
                        self._xCenter[0] = xStart + blobwidth / 2
                        # print('blob detected at: ', xStart, y, ' with center at: ', xCenter[0])
                        return True
                    elif blobwidth > 0:
                        blobwidth = 0
            if blobwidth >= minBlobWidth:
                self._xCenter[0] = xStart + blobwidth / 2
                # print('blob detected at: ', xStart, y, ' with center at: ', xCenter[0])
                return True

        return False

    def applicable(self, distances, noDetectionDistance, accel, image, stepCounter, newImage):
        """
        :param see method in Behaviour class
        :return:  boolean
            behaviour is applicable in current situation
        """

        return self.detectBox(image) #and all(np.greater(distances[1:5], 0.25 * noDetectionDistance * np.ones(4)))



    def calculateMotorValues(self, distances, noDetectionDistance, accel, image, stepCounter, newImage):
        """
        :param see method in Behaviour class
        :return:  (float,float)
            left and right motor velocity
        """

        err = (float(self._resolX)/2 - self._xCenter[0])/float(self._resolX)
        leftMotor = (-0.5*float(err) + 0.75)*self._maxVel/2.0
        rightMotor = (0.5*float(err) + 0.75)*self._maxVel/2.0

        return leftMotor, rightMotor



class PushBox(Behaviour):

    def __init__(self):
        super(PushBox, self).__init__("pushBox")

        self._boxPushingMatrix = np.asarray([[0, 0, 0.0, 1.0],
                                             [1.0, 0.0, 0, 0]])

        self._baseVelocity = np.asarray([self._maxVel/2.0, self._maxVel/2.0])




    def applicable(self, distances, noDetectionDistance, accel, image, stepCounter, newImage):
        """
        :param see method in Behaviour class
        :return:  boolean
            behaviour is applicable in current situation
        """
        return not all(np.greater(distances[1:5], 0.25 * noDetectionDistance * np.ones(4)))


    def calculateMotorValues(self, distances, noDetectionDistance, accel, image, stepCounter, newImage):
        """
        :param see method in Behaviour class
        :return:  (float,float)
            left and right motor velocity
        """
        [leftMotor, rightMotor] = self._baseVelocity + self._maxVel*self._boxPushingMatrix.dot(self.normalizeDistances(distances[1:5], noDetectionDistance))
        return leftMotor, rightMotor



class UnwedgeFromBox(Behaviour):

    def __init__(self):
        super(UnwedgeFromBox, self).__init__("unwedgeBox")
        self._unwedging = False  #special behaviour: needs a memory which is untypical for subsumption behaviours


    def applicable(self, distances, noDetectionDistance, accel, image, stepCounter, newImage):
        """
        :param see method in Behaviour class
        :return:  boolean
            behaviour is applicable in current situation
        """

        if  self._unwedging and all(np.greater(distances[1:5], 0.25 * noDetectionDistance * np.ones(4))): #got it unwedged
            self._unwedging = False

        if (not self._unwedging) and (stepCounter > 5) and (accel[0] > 0.05):  #first time bumping with box against wall
            self._unwedging = True

        return self._unwedging


    def calculateMotorValues(self, distances, noDetectionDistance, accel, image, stepCounter, newImage):
        """
        :param see method in Behaviour class
        :return:  (float,float)
            left and right motor velocity
        """
        leftMotor, rightMotor = self._maxVel, -self._maxVel
        return leftMotor, rightMotor



def main():

    image = I.new("RGB", (resolX, resolY), "white")

    robot = EPuckVRep('ePuck', port=19999, synchronous=False)

    robot.enableCamera()
    robot.enableAllSensors()
    robot.setSensesAllTogether(True)  # we want fast sensing, so set robot to sensing mode where all sensors are sensed

    noDetectionDistance = 0.05 * robot.getS()  # maximum distance that proximity sensors of ePuck may sense

    behaviours = [UnwedgeFromBox(), PushBox(), ApproachBox(resolX, resolY), SearchBox()]  #subsumption behaviour list, order defines priorities!

    stepCounter = 0

    # main sense-act cycle
    while robot.isConnected():
        stepCounter += 1
        if stepCounter%10 == 0 and (behaviour.getState() == 'searchBox' or behaviour.getState() == 'approachBox'):
            image = robot.getCameraImage()
            newImage = True
        else:
            newImage = False

        robot.fastSensingOverSignal()
        distVector = robot.getProximitySensorValues()
        acceleration = robot.getAccelerometerValues()

        behaviour = next(behavior for behavior in behaviours
                                       if behavior.applicable(distVector, noDetectionDistance, acceleration[0:2], image,
                                            stepCounter, newImage))

        print("behaviour: ", behaviour.getState())

        leftMotor, rightMotor = behaviour.calculateMotorValues(distVector, noDetectionDistance,
                                                                acceleration[0:2], image, stepCounter, newImage)

        robot.setMotorSpeeds(leftMotor, rightMotor)
        time.sleep(0.05)

    robot.disconnect()



if __name__ == '__main__':
    main()
