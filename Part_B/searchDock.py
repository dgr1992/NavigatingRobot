from behaviour import Behaviour

class SearchDock(Behaviour):

    def __init__(self):
        super(SearchDock, self).__init__("searchDock")

    def applicable(self, image, isNewImage, currentPose, goalPose, kRoh, kAlpha, kBeta):
        """
        :param see method in Behaviour class
        :return:  boolean
            behaviour is applicable in current situation
        """
        return True


    def calculateMotorValues(self, image, isNewImage, currentPose, goalPose, kRoh, kAlpha, kBeta):
        """
        :param see method in Behaviour class
        :return:  (float,float)
            left and right motor velocity
        """
        return 1, 0.25 #do not turn on possition to keep odometrie error as small as possible