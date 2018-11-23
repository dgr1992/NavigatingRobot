from behaviour import Behaviour

class ApproachDock(Behaviour):

    def __init__(self, resolutionX, resolutionY):
        self._xCenter = [-1]
        self._resolX = resolutionX
        self._resolY = resolutionY
        super(ApproachDock, self).__init__("approachDock")

    def detectDock(self, image):
        """
        looks in current image for a black blob on a red background, from left to right
        :param
                image: PIL.Image
                    a rgb image with black blobs on red background

        :return: true,  if black blob found.
                side-effect: if blob found, the function calculates xCenter: [int]
                    the center of the image: result of the function

        """
        #ToDo check this
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

    def applicable(self, image, isNewImage):
        """
        :param see method in Behaviour class
        :return:  boolean
            behaviour is applicable in current situation
        """

        return self.detectDock(image)


    def calculateMotorValues(self, image, isNewImage):
        """
        :param see method in Behaviour class
        :return:  (float,float)
            left and right motor velocity
        """

        err = (float(self._resolX)/2 - self._xCenter[0])/float(self._resolX)
        leftMotor = (-0.5*float(err) + 0.75)*self._maxVel/2.0
        rightMotor = (0.5*float(err) + 0.75)*self._maxVel/2.0

        return leftMotor, rightMotor
