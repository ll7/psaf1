import cv2
import numpy as np
import rospy

from psaf_abstraction_layer.sensors.RGBCamera import RGBCamera
from psaf_abstraction_layer.sensors.SegmentationCamera import SegmentationCamera,Tag
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

        self.ratio_range_per_mille = range(5, 150)

        self.canny_threshold = 100

        self.camera = SegmentationCamera(role_name=role_name,id="front")
        self.camera.set_on_image_listener(self.__on_new_image)

    @classmethod
    def __heuristic_calc_distance_by_y(cls, y:float)->float:
        """
        Calculates the distance in meters by using the y coordinate of the detected object
        :param y: the y coordinate of the stop line center
        :return:
        """
        a_4= 4341.4
        a_3 = - 13727
        a_2 = 16300
        a_1 = - 8635.5
        a_0 = 1727.4
        return a_4*y**4+a_3*y**3+a_2*y**2+a_1*y+a_0

    def __on_new_image(self, segmentation_image, time):
        (H, W) = segmentation_image.shape[:2]

        # List oif detected elements
        detected = []

        filter_image = SegmentationCamera.filter_for_tags(segmentation_image,{Tag.RoadLine})

        canny_output = cv2.Canny(filter_image, self.canny_threshold, self.canny_threshold * 2)
        contours, _ = cv2.findContours(canny_output, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        boxes = []
        for i, c in enumerate(contours):
            contours_poly = cv2.approxPolyDP(c, 3, True)
            x1, y1, w, h = cv2.boundingRect(contours_poly)
            boxes.append(np.array([x1, y1, w, h]) / np.array([W, H, W, H]))

        for x1, y1, w, h in boxes:
            if y1>0.3 and x1+w< 0.9:
                if int(h/w*1000) in self.ratio_range_per_mille and w> 0.3:
                    detected.append(DetectedObject(x1, y1, w, h, distance=self.__heuristic_calc_distance_by_y(y1), label=Labels.StopLine))

        self.inform_listener(detected)


if __name__ == "__main__":
    # Show case code
    def show_image(title, image):
        max_width, max_height = 1200, 800

        limit = (max_height, max_width)
        fac = 1.0
        if image.shape[0] > limit[0]:
            fac = limit[0] / image.shape[0]
        if image.shape[1] > limit[1]:
            fac = limit[1] / image.shape[1]
        image = cv2.resize(image, (int(image.shape[1] * fac), int(image.shape[0] * fac)))
        # show the output image
        cv2.imshow(title, cv2.cvtColor(image, cv2.COLOR_RGB2BGR))
        cv2.waitKey(1)


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
                text = "{:.1f}m: x:{:.3f}-y:{:.3f}".format(element.distance, element.x,element.y)
                cv2.putText(image, text, (x - 5, y - 5), cv2.FONT_HERSHEY_SIMPLEX, 1, color, 2)

        show_image("Stop line detection", image)


    def on_detected(detected_list):
        global detected_r
        detected_r = detected_list


    cam = RGBCamera()

    cam.set_on_image_listener(store_image)

    s = StopLineDetector()
    s.set_on_detection_listener(on_detected)
    rospy.spin()