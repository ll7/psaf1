import math

import cv2
import numpy as np
import rospy

from psaf_abstraction_layer.sensors.SegmentationCamera import SegmentationCamera, Tag
from psaf_perception.detectors.AbstractDetector import AbstractDetector, DetectedObject, Labels


class StopLineDetector(AbstractDetector):
    """
    Detector for stop lines
    """

    def __init__(self, role_name: str = "ego_vehicle"):
        """
        Init the speed sign detector
        :param role_name: the name of the vehicle to access the cameras
        """
        super().__init__()

        self.logger_name = "TrafficLightDetector"

        self.ratio_range_per_mille = range(5, 500)

        self.canny_threshold = 100

        self.camera = SegmentationCamera(role_name=role_name, id="front")
        self.camera.set_on_image_listener(self.__on_new_image)

    @classmethod
    def __heuristic_calc_distance_by_y(cls, y: float) -> float:
        """
        Calculates the distance in meters by using the y coordinate of the detected object
        :param y: the y coordinate of the stop line center
        :return:
        """
        a_4 = 4341.4
        a_3 = - 13727
        a_2 = 16300
        a_1 = - 8635.5
        a_0 = 1727.4
        return a_4 * y ** 4 + a_3 * y ** 3 + a_2 * y ** 2 + a_1 * y + a_0

    def __on_new_image(self, segmentation_image, time):
        (H, W) = segmentation_image.shape[:2]

        # Scale image to improve performance
        max_width, max_height = 600, 400

        limit = (max_height, max_width)
        fac = 1.0
        if H > limit[0]:
            fac = limit[0] / H
        if W > limit[1]:
            fac = limit[1] / W
        segmentation_image = cv2.resize(segmentation_image, (int(W * fac), int(H * fac)))
        H *= fac
        W *= fac
        # List oif detected elements
        detected = []

        filter_image = SegmentationCamera.filter_for_tags(segmentation_image, {Tag.RoadLine})

        edges = cv2.Canny(filter_image, 50, 150, apertureSize=3)
        lines = cv2.HoughLinesP(edges, 1, np.pi /180 , 30, minLineLength=W*0.25,maxLineGap=20)

        if lines is not None:
            for i, line in enumerate(lines):

                x1_abs, y1_abs, x2_abs, y2_abs = line[0]
                x1, y1, x2, y2 = np.array(line[0]) / np.array([W, H, W, H])

                w= x2-x1
                h= y2-y1
                angle = math.degrees(math.atan2(h,w))

                if x2 >0.5 and w > 0.1 and -30 < angle < 30 :
                    detected.append(DetectedObject(x1, y1, w, h, distance=self.__heuristic_calc_distance_by_y(y1+h/2),
                                                   label=Labels.StopLine))

        self.inform_listener(time, detected)


if __name__ == "__main__":
    from psaf_abstraction_layer.sensors.RGBCamera import RGBCamera
    from psaf_perception.perception_util import show_image


    # Show case code

    rospy.init_node("StopLineTest")

    detected_r = None


    def store_image(image, _):
        global detected_r

        H, W = image.shape[:2]

        if detected_r is not None:
            for element in detected_r:
                # extract the bounding box coordinates
                (x, y) = (int(element.x * W), int(element.y * H))
                (w, h) = (int(element.w * W), int(element.h * H))
                # draw a bounding box rectangle and label on the image
                color = (255, 0, 0)
                cv2.rectangle(image, (x, y), (x + w, y + h), color, 2)
                text = "{:.1f}m: x:{:.3f}-y:{:.3f}".format(element.distance, element.x, element.y)
                cv2.putText(image, text, (x - 5, y - 5), cv2.FONT_HERSHEY_SIMPLEX, 1, color, 2)

        show_image("Stop line detection", image)


    def on_detected(_, detected_list):
        global detected_r
        detected_r = detected_list


    cam = RGBCamera()

    cam.set_on_image_listener(store_image)

    s = StopLineDetector()
    s.set_on_detection_listener(on_detected)
    rospy.spin()
