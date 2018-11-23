from behaviour import Behaviour
import numpy as np
import math

class Docking(Behaviour):
    reachedTarget = False

    def __init__(self, robot, goalPoseMatrix, controlParameterMatrix):
        """
        :param
            robot: robot object
            goalPoseMatrix: Matrix containing the goal pose [[xGoal], [yGoal], [zetaGoal]]
            controlParameterMatrix: [[kRoh][kAlpha][kBeta]]
        """
        super(Docking, self).__init__("Docking")
        self.robot = robot
        self.wheelRadius = robot._wheelDiameter/2
        self.wheelDistance = robot._wheelDistance
        self.K =self.createControlMatrix(controlParameterMatrix[0][0],controlParameterMatrix[1][0],controlParameterMatrix[2][0])
        self.oldencval = robot.getWheelEncodingValues()
        self.currentPose = self.getInitialPose()
        self.goalPose = goalPoseMatrix

    def applicable(self, image, isNewImage):
        """
        :param see method in Behaviour class
        :return:  boolean
            behaviour is applicable in current situation
        """
        return False


    def calculateMotorValues(self, image, isNewImage):
        """
        :param see method in Behaviour class
        :return:  (float,float)
            left and right motor velocity
        """
        self.currentPose = self.updatePose(self.currentPose)
        #Get distance to goal
        deltaX = math.fabs(self.currentPose[0][0] - self.goalPose[0][0])
        deltaY = math.fabs(self.currentPose[1][0] - self.goalPose[1][0])
        deltaTheta = math.fabs(self.currentPose[2][0] - self.goalPose[2][0])

        if self.reachedTarget or (deltaX < 0.003  and deltaY < 0.002 and deltaTheta < 0.1) : 
            reachedTarget = True
            motorSpeed = np.matrix([[0],[0]])
        else:
            #Calculate the motor speeds so the robot moves torwards the goal
            polar = self.polarTransf(self.currentPose, self.goalPose)
            motorSpeed = self.K * polar
        return motorSpeed[1][0], motorSpeed[0][0]

    def updatePose(self,currentPose):
        #Calculate the driven distance of the left and right wheel
        encval = self.robot.getWheelEncodingValues()
        dsl, dsr, self.oldencval = self.calcDriven(encval, self.oldencval, self.wheelRadius)
        #Calculate the pose
        pose = self.calcCurrentPos(currentPose, self.wheelDistance, dsr , dsl)
        return pose

    def createControlMatrix(self,kRoh,kAlpha,kBeta):
        # Define the control matrix
        c00 = kRoh/self.wheelRadius
        c01 = (kAlpha*self.wheelDistance)/self.wheelRadius
        c02 = (kBeta*self.wheelDistance)/self.wheelRadius
        c10 = kRoh/self.wheelRadius
        c11 = -(kAlpha*self.wheelDistance)/self.wheelRadius
        c12 = -(kBeta*self.wheelDistance)/self.wheelRadius
        K = np.matrix([[c00,c01,c02],[c10,c11,c12]])
        return K

    def calcDriven(self,encval, oldencval, radius):
        leftdist = (encval[0] + 2*math.pi - oldencval[0]) % (2*math.pi) * radius
        rightdist = (encval[1] + 2*math.pi - oldencval[1]) % (2*math.pi) * radius

        return leftdist, rightdist, encval

    def calcCurrentPos(self,oldPos, wheelBase, drivenSr, drivenSl):
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

    def polarTransf(self,current, goal):
        deltaX = goal[0][0] - current[0]
        deltaY = goal[1][0] - current[1]
        roh = math.sqrt(deltaX**2 + deltaY**2)
        alpha = - current[2] + math.atan2(deltaY,deltaX)
        beta = - current[2] - alpha
        return np.matrix([[roh],[alpha],[beta]])

    def getInitialPose(self):
        # get start Position and convert to numpy-matrix
        temp = self.robot._getPose()
        xStart = temp[0]
        yStart = temp[1]
        zetaStart = temp[2]
        return np.matrix([[xStart],[yStart], [zetaStart]])