
class DetectedObject:
    """
    Data class for detected elements
    """

    def __init__(self, x: int = 0, y: int = 0, w: int = 0, h: int = 0, label: str = "UNKNOWN",
                 confidence: float = 0.01):
        """

        :param x: x coord in image
        :param y: y coord in image
        :param h: height of bounding box
        :param w: weight of bounding box
        :param label: the class label name
        :param confidence: the confidence value
        """
        self.x = x
        self.y = y
        self.h = h
        self.w = w
        self.label = label
        self.confidence = confidence


class AbstractDetector:

    def __init__(self):
        self.__listener = None

    def inform_listener(self,detected_list):
        if self.__listener is not None:
            self.__listener(detected_list)

    def set_on_detection_listener(self, func):
        """
        Set function to be called with detected bounding boxes
        :param func: the function
        :return: None
        """
        self.__listener = func;