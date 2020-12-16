from enum import Enum, Flag
from typing import Set


class LabelGroups(Enum):
    """
    Enum to group the Labels
    """
    Unknown = 0
    Speed = 1
    TrafficLight = 2
    RightOfWay = 3


LabelGroupSet = Set[LabelGroups]


class Labels(Enum):
    """
    Enum that stores the available perception tags used to communicate between the components
    """

    def __init__(self, id: int, label: str, groups: LabelGroupSet = {}):
        self._value_ = id
        self._label = label
        self._groups = groups

    @property
    def groups(self) -> LabelGroupSet:
        """
        Returns the set of groups
        :return:
        """
        return self._groups

    @property
    def label_text(self) -> str:
        """
        Returns the label name
        :return: ł
        """
        return self._label

    def __str__(self):
        return self.label_text

    Unlabeled = (0, "Unlabeled")
    Stop = (1, "Stop", {LabelGroups.RightOfWay})
    SpeedLimit30 = (2, "Speed limit 30", {LabelGroups.Speed})
    Speed30 = (3, "Speed 30", {LabelGroups.Speed})
    Speed60 = (4, "Speed 60", {LabelGroups.Speed})
    Speed90 = (5, "Speed 90", {LabelGroups.Speed})
    TrafficLightUnknown = (6, "Traffic light unknown", {LabelGroups.TrafficLight, LabelGroups.RightOfWay})
    TrafficLightRed = (7, "Traffic light red",{LabelGroups.TrafficLight, LabelGroups.RightOfWay})
    TrafficLightYellow = (8, "Traffic light yellow", {LabelGroups.TrafficLight, LabelGroups.RightOfWay})
    TrafficLightYellowRed = (9, "Traffic light red-yellow", {LabelGroups.TrafficLight, LabelGroups.RightOfWay})
    TrafficLightGreen = (10, "Traffic light green", {LabelGroups.TrafficLight, LabelGroups.RightOfWay})
    TrafficLightOff = (11, "Traffic light off", {LabelGroups.TrafficLight, LabelGroups.RightOfWay})

    Other = (99, "Other")


class DetectedObject:
    """
    Data class for detected elements
    """

    def __init__(self, x: int = 0, y: int = 0, w: int = 0, h: int = 0, label: Labels = 0,
                 confidence: float = 0.01):
        """

        :param x: x coord in image
        :param y: y coord in image
        :param h: height of bounding box
        :param w: weight of bounding box
        :param label: the class label
        :param confidence: the confidence value
        """
        self.x = x
        self.y = y
        self.h = h
        self.w = w
        self.label = label
        self.confidence = confidence


class AbstractDetector:
    """Abstract detector class for all Detector instances"""

    def __init__(self):
        self.__listener = None

    def inform_listener(self, detected_list):
        """
        Informs all listeners about the new list of detected objects
        :param detected_list: the list of detected objects
        :return: None
        """
        if self.__listener is not None:
            self.__listener(detected_list)

    def set_on_detection_listener(self, func):
        """
        Set function to be called with detected bounding boxes
        :param func: the function
        :return: None
        """
        self.__listener = func;
