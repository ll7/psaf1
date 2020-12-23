import math
from abc import abstractmethod
from typing import Set, Tuple, List, Callable

import cv2
import numpy as np
import rospy, rospkg
import os
from std_msgs.msg import Time

from psaf_abstraction_layer.sensors.DepthCamera import DepthCamera
from psaf_abstraction_layer.sensors.RGBCamera import RGBCamera
from psaf_abstraction_layer.sensors.SegmentationCamera import SegmentationCamera, Tag as SegmentationTag


# TODO a very ugly name just change it or leave it. I'm just a comment so why should you do what I say
class CameraDataCombinator:

    def __init__(self, role_name: str = "ego_vehicle", time_threshold=0.1, visible_tags: Set[SegmentationTag] = None):
        super().__init__()

        self.visible_tags = visible_tags

        self.confidence_min = 0.70

        # Threshold between two image in seconds -> smaller is better but makes it harder to find partners
        self.time_threshold = time_threshold

        # Init camera
        self.depth_camera = DepthCamera(role_name, "front")
        self.depth_camera.set_on_image_listener(self.__on_depth_image)
        self.depth_images: List[Tuple[Time, np.ndarray]] = []

        self.rgb_camera = RGBCamera(role_name, "front")
        self.rgb_camera.set_on_image_listener(self.__on_rgb_image_update)
        self.rgb_images: List[Tuple[Time, np.ndarray]] = []

        self.segmentation_camera = SegmentationCamera(role_name, "front")
        self.segmentation_camera.set_on_image_listener(self.__on_segment_image_update)

        self.__listener = None

    def __on_depth_image(self, image, time):
        self.depth_images.append((time, image))

    def __on_rgb_image_update(self, image, time):
        self.rgb_images.append((time, image))

    def __time_difference(self, time_a, time_b) -> float:
        """
        Calculate the difference of to time stamps in seconds.
        :param time_a: time stamp A
        :param time_b: time stamp B
        :return: the difference in seconds
        """
        if time_a is None or time_b is None:
            return math.nan
        return float(time_a.secs - time_b.secs + (time_a.nsecs - time_b.nsecs) * (10 ** -9))

    def __on_segment_image_update(self, image, time):

        # closest_depth_image
        depth_time, depth_image = next(iter(
            sorted(
                filter(lambda x: abs(self.__time_difference(x[0], time)) <= self.time_threshold, self.depth_images),
                key=lambda s: float(s[0].secs + s[0].nsecs * (10 ** -9)))
        ), (None, None))

        # Drop all images in list that are older than the best match
        if depth_time is not None:
            for old in filter(lambda x: x[0] <= depth_time, self.depth_images):
                self.depth_images.remove(old)

        # closest rgb_image
        rgb_time, rgb_image = next(iter(
            sorted(
                filter(lambda x: abs(self.__time_difference(x[0], time)) <= self.time_threshold, self.rgb_images),
                key=lambda s: float(s[0].secs + s[0].nsecs * (10 ** -9)))
        ), (None, None))

        # Drop all images in list that are older than the best match
        if rgb_time is not None:
            for old in filter(lambda x: x[0] <= rgb_time, self.rgb_images):
                self.rgb_images.remove(old)

        # stop if not corresponding images were found
        if rgb_image is not None and depth_image is not None:
            # Filter segmentation camera image for the given tags
            if self.visible_tags is not None:
                image = SegmentationCamera.filter_for_tags(image, self.visible_tags)

            # Call abstract method
            if self.__listener is not None:
                self.__listener(image,rgb_image,depth_image,time)


    def set_on_image_data_listener(self, func:Callable[[np.ndarray, np.ndarray, np.ndarray, Time], None]):
        """
        Set function to be called with image data
        :param func: the function ( segmentation_image, rgb_image, depth_image, time) -> None
        :return: None
        """
        self.__listener = func


# Show case code
if __name__ == "__main__":
    rospy.init_node("DetectionTest")

    detected_r = None


    def store_image(image, _):
        global detected_r
        H, W = image.shape[:2]

        # if detected_r is not None:
        #     for element in detected_r:
        #         # extract the bounding box coordinates
        #         (x, y) = (int(element.x * W), int(element.y * H))
        #         (w, h) = (int(element.w * W), int(element.h * H))
        #         # draw a bounding box rectangle and label on the image
        #         color = (255, 0, 0)
        #         cv2.rectangle(image, (x, y), (x + w, y + h), color, 2)
        #         text = "{} in {:.1f}m: {:.4f}".format(element.label.label_text, element.distance, element.confidence)
        #         cv2.putText(image, text, (x - 5, y - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.75, color, 2)
        #
        # # show the output image
        # cv2.imshow("RGB", cv2.cvtColor(image, cv2.COLOR_RGB2BGR))
        # cv2.waitKey(1)


    def on_detected(detected_list):
        global detected_r
        detected_r = detected_list


    cam = RGBCamera()

    cam.set_on_image_listener(store_image)

    s = CameraDataCombinator()
    s.set_on_image_data_listener(on_detected)
    rospy.spin()
