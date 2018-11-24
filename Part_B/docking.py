from behaviour import Behaviour
import numpy as np
import math

class Docking(Behaviour):

    def __init__(self, wheelRadius, wheelDistance, resolutionX, resolutionY):
        """
        :param
            robot: robot object
            goalPoseMatrix: Matrix containing the goal pose [[xGoal], [yGoal], [zetaGoal]]
            controlParameterMatrix: [[kRoh][kAlpha][kBeta]]
        """
        super(Docking, self).__init__("Docking")
        #self.K =self.createControlMatrix(controlParameterMatrix[0][0],controlParameterMatrix[1][0],controlParameterMatrix[2][0],wheelRadius, wheelDistance)
        self.reachedTarget = False
        self.wheelRadius = wheelRadius
        self.wheelDistance = wheelDistance  
        self._resolX = resolutionX
        self._resolY = resolutionY 
        self._xCenter = [-1]

    def applicable(self, image, isNewImage, currentPose, goalPose, kRoh, kAlpha, kBeta):
        """
        :param see method in Behaviour class
        :return:  boolean
            behaviour is applicable in current situation
        """
        minBlobWidth = 25
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
                    #print blobwidth
                    if blobwidth >= minBlobWidth:
                        return True
                    elif blobwidth > 0:
                        blobwidth = 0
            if blobwidth >= minBlobWidth:
                #print blobwidth
                self._xCenter[0] = xStart + blobwidth / 2
                return True

        return False

    def calculateMotorValues(self, image, isNewImage, currentPose, goalPose, kRoh, kAlpha, kBeta):
        """
        :param see method in Behaviour class
        :return:  (float,float)
            left and right motor velocity
        """
        #Get distance to goal
        deltaX = math.fabs(currentPose[0][0] - goalPose[0][0])
        deltaY = math.fabs(currentPose[1][0] - goalPose[1][0])
        deltaTheta = math.fabs(currentPose[2][0] - goalPose[2][0])

        K = self.createControlMatrix(kRoh,kAlpha,kBeta,self.wheelRadius, self.wheelDistance)

        if self.reachedTarget or (deltaX < 0.003  and deltaY < 0.002 and deltaTheta < 0.1) : 
            self.reachedTarget = True
            motorSpeed = np.matrix([[0],[0]])
        else:
            #Calculate the motor speeds so the robot moves torwards the goal
            polar = self.polarTransf(currentPose, goalPose)
            motorSpeed = K * polar
        return motorSpeed[1][0], motorSpeed[0][0]

    def createControlMatrix(self,kRoh,kAlpha,kBeta, wheelRadius, wheelDistance):
        # Define the control matrix
        c00 = kRoh/wheelRadius
        c01 = (kAlpha*wheelDistance)/wheelRadius
        c02 = (kBeta*wheelDistance)/wheelRadius
        c10 = kRoh/wheelRadius
        c11 = -(kAlpha*wheelDistance)/wheelRadius
        c12 = -(kBeta*wheelDistance)/wheelRadius
        K = np.matrix([[c00,c01,c02],[c10,c11,c12]])
        return K

    def polarTransf(self,current, goal):
        deltaX = goal[0][0] - current[0]
        deltaY = goal[1][0] - current[1]
        roh = math.sqrt(deltaX**2 + deltaY**2)
        alpha = - current[2] + math.atan2(deltaY,deltaX)
        beta = - current[2] - alpha
        return np.matrix([[roh],[alpha],[beta]])