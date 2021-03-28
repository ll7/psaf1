from enum import Enum, EnumMeta

import numpy
import numpy as np
import rospy
import cv2
from sensor_msgs.msg import Image
from std_msgs.msg import Time
from cv_bridge.core import CvBridge
from typing import Set, Callable


class Tag(Enum):
    """
    Enum that stores the colors for the available tags inside the segmentation camera
    """

    def __init__(self, id, label, color):
        self._value_ = id
        self.color = color
        self.label = label

    @property
    def get_color(self):
        return self.color

    @property
    def get_label(self):
        return self.label

    """
    For details go to https://carla.readthedocs.io/en/latest/ref_sensors/#semantic-segmentation-camera
    """
    Unlabeled = (0, "Unlabeled", (0, 0, 0))
    Building = (1, "Building", (70, 70, 70))
    Fence = (2, "Fence", (100, 40, 40))
    Other = (3, "Other", (55, 90, 80))
    Pedestrian = (4, "Pedestrian", (220, 20, 60))
    Pole = (5, "Pole", (153, 153, 153))
    RoadLine = (6, "RoadLine", (157, 234, 50))
    Road = (7, "Road", (128, 64, 128))
    SideWalk = (8, "SideWalk", (244, 35, 232))
    Vegetation = (9, "Vegetation", (107, 142, 35))
    Vehicles = (10, "Vehicles", (0, 0, 142))
    Wall = (11, "Wall", (102, 102, 156))
    TrafficSign = (12, "TrafficSign", (220, 220, 0))
    Sky = (13, "Sky", (70, 130, 180))
    Ground = (14, "Ground", (81, 0, 81))
    RailTrack = (16, "RailTrack", (230, 150, 140))
    GuardRail = (17, "GuardRail", (180, 165, 180))
    TrafficLight = (18, "TrafficLight", (250, 170, 30))
    Static = (19, "Static", (110, 190, 160))
    Dynamic = (20, "Dynamic", (170, 120, 50))
    Water = (21, "Water", (45, 60, 150))
    Terrain = (22, "Terrain", (145, 170, 100))


class SegmentationCamera:
    """
    The segmentation camera interface that allows us to  access the segmentation camera
    """

    def __init__(self, role_name: str = "ego_vehicle", id: str = "front",queue_size=None):

        self.image = None
        self.bridge = CvBridge()
        self.__subscriber = rospy.Subscriber(
            "/carla/{}/camera/semantic_segmentation/{}/image_segmentation".format(role_name, id), Image,
            self.__update_image,queue_size=queue_size)

        self.__listener = None

    def __update_image(self, image_msg: Image):
        """
        Internal method to update the position data
        :param image_msg: the message
        :return: None
        """
        self.image = self.bridge.imgmsg_to_cv2(image_msg, desired_encoding='rgb8')

        if self.__listener != None:
            self.__listener(self.image,image_msg.header.stamp)

    def get_image(self):
        """
        Return the current segmentation image
        :return:the current image
        """
        return self.image

    def set_on_image_listener(self, func:Callable[[numpy.ndarray,Time],None]):
        """
        Set function to be called with the segmentation image as parameter
        :param func: the function
        :return: None
        """
        self.__listener = func;

    @classmethod
    def filter_for_tags(cls, image: np.ndarray, tags: Set[Tag]) -> np.ndarray:
        """
        Filters the given image for the given set of tags
        :param image: the image that will be filtered
        :param tags: the set of tags
        :return: the filtered image
        """
        masks = []
        for tag in tags:
            color = tag.color;
            red_mask = image[:, :, 0] == color[0]
            green_mask = image[:, :, 1] == color[1]
            blue_mask = image[:, :, 2] == color[2]
            masks.append(np.logical_and(red_mask, green_mask, blue_mask))

        final_mask = np.logical_not(np.logical_or.reduce(masks))
        image[final_mask] = 255
        return image


# Show case code
if __name__ == "__main__":
    rospy.init_node("SegmentationTest")


    def show_image(image,_):
        (H, W) = image.shape[:2]
        max_width = 800
        new_width = int(max_width)
        new_height = int(H * max_width / W)
        image = cv2.resize(image, (new_width,new_height,), interpolation=cv2.INTER_AREA)
        image = SegmentationCamera.filter_for_tags(image, {Tag.RoadLine})
        image = cv2.cvtColor(image,cv2.COLOR_RGB2GRAY)
        bw_image = cv2.threshold(image, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)[1]
        # show the output image
        cv2.imshow("Segmentation", bw_image)
        cv2.waitKey(1)


    cam = SegmentationCamera(id="street")

    cam.set_on_image_listener(show_image)
    rospy.spin()
