from behaviour import Behaviour

class SearchDock(Behaviour):

    def __init__(self):
        super(SearchDock, self).__init__("searchDock")

    def applicable(self, image, isNewImage):
        """
        :param see method in Behaviour class
        :return:  boolean
            behaviour is applicable in current situation
        """
        return True


    def calculateMotorValues(self, image, isNewImage):
        """
        :param see method in Behaviour class
        :return:  (float,float)
            left and right motor velocity
        """
        return 0.1, -0.1    # turn clockwise, since box detection in image searches from left to right,
                            # and we want to detect the same box again, not a second one appearing now