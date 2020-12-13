#!/usr/bin/env python
import asyncio
from typing import List

import rospy
from psaf_messages.msg import TrafficSignInfo, SpeedSign, StopMark

from psaf_perception.AbstractDetector import DetectedObject, Labels, LabelGroups
from psaf_perception.SpeedSignDetector import SpeedSignDetector, CONV_FAC_MPH_TO_KMH
from psaf_perception.StopMarkDetector import StopMarkDetector


class DetectionService:
    """Stores the detected object from all detector"""

    def __init__(self):
        self.detectors = {}
        rospy.init_node("DetectionService")
        role_name = rospy.get_param("role_name", "ego_vehicle")
        # Add detector here
        self.detectors.update({"speed": SpeedSignDetector(role_name)})
        self.detectors.update({"stop": StopMarkDetector(role_name)})

        # Data
        # store all speed signs
        self.speed_signs : List[SpeedSign]= []
        # store all stop marks
        self.stop_marks : List[StopMark]= []

        # Ros components
        self.traffic_sign_publisher = rospy.Publisher("/psaf/perception/traffic_signs", TrafficSignInfo,
                                                      queue_size=1)  # TODO replace string with correct message type
        # Helpers
        self.publish_timer = rospy.Timer(rospy.Duration(0.1), self.periodic_update)
        self.lock = asyncio.Lock()

        # Collect detection
        self.detectors["speed"].set_on_detection_listener(self.__on_new_speed_Sign)
        self.detectors["stop"].set_on_detection_listener(self.__on_new_stop)

    def __on_new_speed_Sign(self, detected: List[DetectedObject]):
        self.speed_signs.clear()
        for object in detected:
            x_center = object.x + object.w / 2
            y_center = object.y + object.h / 2
            if LabelGroups.Speed in object.label.groups:
                limit = 30
                if object.label == Labels.Speed30:
                    limit = 30
                elif object.label == Labels.Speed60:
                    limit = 60
                elif object.label == Labels.Speed90:
                    limit = 90
                msg = SpeedSign()
                msg.x = x_center
                msg.y = y_center
                msg.limit = limit/CONV_FAC_MPH_TO_KMH
                self.speed_signs.append(msg)

    def __on_new_stop(self, detected: List[DetectedObject]):
        self.stop_marks.clear()
        for object in detected:
            x_center = object.x + object.w / 2
            y_center = object.y + object.h / 2
            if object.label == Labels.Stop:
                msg = StopMark()
                msg.x = x_center
                msg.y = y_center
                self.stop_marks.append(msg)

    def periodic_update(self, event):
        """
        Periodically sends an update about the currently detected objects
        :param event: the timer event ( it is ignored)
        :return: None
        """
        signMsg = TrafficSignInfo()
        signMsg.header.stamp = rospy.Time.now()
        signMsg.header.frame_id = 'DetectionServiceTrafficSigns'
        signMsg.speedSigns.extend(self.speed_signs)
        signMsg.stopMarks.extend(self.stop_marks)
        self.traffic_sign_publisher.publish(signMsg)


if __name__ == "__main__":
    service = DetectionService()
    rospy.spin()
