from enum import Enum, Flag
from typing import Set, List, Callable


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

    def __init__(self, id: int, label: str, groups: LabelGroupSet = None):
        self._value_ = id
        self._label = label
        if groups is None:
            groups = {}
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
        :return:
        """
        return self._label

    def __str__(self):
        return self.label_text

    Unlabeled = (0, "Unlabeled")
    StopSurfaceMarking = (1, "Stop marking", {LabelGroups.RightOfWay})
    StopSign = (2, "Stop sign", {LabelGroups.RightOfWay})
    SpeedLimit30 = (10, "Speed limit 30", {LabelGroups.Speed})
    SpeedLimit40 = (11, "Speed limit 40", {LabelGroups.Speed})
    SpeedLimit60 = (12, "Speed limit 60", {LabelGroups.Speed})
    Speed30 = (13, "Speed 30", {LabelGroups.Speed})
    Speed60 = (14, "Speed 60", {LabelGroups.Speed})
    Speed90 = (15, "Speed 90", {LabelGroups.Speed})
    TrafficLightUnknown = (20, "Traffic light unknown", {LabelGroups.TrafficLight, LabelGroups.RightOfWay})
    TrafficLightRed = (21, "Traffic light red",{LabelGroups.TrafficLight, LabelGroups.RightOfWay})
    TrafficLightYellow = (22, "Traffic light yellow", {LabelGroups.TrafficLight, LabelGroups.RightOfWay})
    TrafficLightYellowRed = (23, "Traffic light red-yellow", {LabelGroups.TrafficLight, LabelGroups.RightOfWay})
    TrafficLightGreen = (24, "Traffic light green", {LabelGroups.TrafficLight, LabelGroups.RightOfWay})
    TrafficLightOff = (25, "Traffic light off", {LabelGroups.TrafficLight, LabelGroups.RightOfWay})

    Other = (99, "Other")


class DetectedObject:
    """
    Data class for detected elements
    """

    def __init__(self, x: float = 0, y: float = 0, width: float = 0, height: float = 0, distance:float = 0, label: Labels = 0,
                 confidence: float = 0.01):
        """

        :param x: relative x coord in image
        :param y: relative y coord in image
        :param height: relative height of bounding box
        :param width: relative weight of bounding box
        :param distance: distance in meters. 0 (default) stands for an unknown distance
        :param label: the class label
        :param confidence: the confidence value
        """
        self.x = x
        self.y = y
        self.h = height
        self.w = width
        self.distance = distance
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

    def set_on_detection_listener(self, func:Callable[[List[DetectedObject]],None]):
        """
        Set function to be called with detected objects
        :param func: the function
        :return: None
        """
        self.__listener = func;
