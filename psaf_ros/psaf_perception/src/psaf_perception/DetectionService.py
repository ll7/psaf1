#!/usr/bin/env python
import asyncio
from typing import List

import rospy
from psaf_messages.msg import TrafficSignInfo, SpeedSign, StopMark, TrafficLight

from psaf_perception.AbstractDetector import DetectedObject, Labels, LabelGroups
from psaf_perception.SpeedSignDetector import SpeedSignDetector, CONV_FAC_MPH_TO_KMH
from psaf_perception.StopMarkDetector import StopMarkDetector
from psaf_perception.TrafficLightDetector import TrafficLightDetector

ROOT_TOPIC = "/psaf/perception/"


class DetectionService:
    """Stores the detected object from all detector"""

    def __init__(self):
        self.detectors = {}
        rospy.init_node("DetectionService")
        role_name = rospy.get_param("role_name", "ego_vehicle")
        # Add detector here
        self.detectors.update({"speed": SpeedSignDetector(role_name)})
        self.detectors.update({"stop": StopMarkDetector(role_name)})
        self.detectors.update({"trafficLight": TrafficLightDetector(role_name)})

        # Data
        # store all speed signs
        self.speed_signs: List[SpeedSign] = []
        # store all stop marks
        self.stop_marks: List[StopMark] = []
        # store all traffic lights
        self.trafficLights: List[TrafficLight] = []

        # Ros components
        self.traffic_sign_publisher = rospy.Publisher(ROOT_TOPIC + "traffic_signs", TrafficSignInfo, queue_size=1)
        # Helpers
        self.publish_timer = rospy.Timer(rospy.Duration(0.1), self.periodic_update)
        self.lock = asyncio.Lock()

        # Collect detection
        self.detectors["speed"].set_on_detection_listener(self.__on_new_speed_sign)
        self.detectors["stop"].set_on_detection_listener(self.__on_new_stop)
        self.detectors["trafficLight"].set_on_detection_listener(self.__on_new_traffic_light)

    def __on_new_speed_sign(self, detected: List[DetectedObject]):
        self.speed_signs.clear()
        for object in detected:
            if LabelGroups.Speed in object.label.groups:
                limit = 30
                if object.label == Labels.Speed30:
                    limit = 30
                elif object.label == Labels.Speed60:
                    limit = 60
                elif object.label == Labels.Speed90:
                    limit = 90
                msg = SpeedSign()
                msg.x = object.x + object.w / 2
                msg.y = object.y + object.h / 2
                msg.distance = object.distance
                msg.limit = int(limit / CONV_FAC_MPH_TO_KMH)
                self.speed_signs.append(msg)

    def __on_new_stop(self, detected: List[DetectedObject]):
        self.stop_marks.clear()
        for object in detected:
            if object.label == Labels.Stop:
                msg = StopMark()
                msg.x = object.x + object.w / 2
                msg.y = object.y + object.h / 2
                self.stop_marks.append(msg)

    def __on_new_traffic_light(self, detected: List[DetectedObject]):
        self.trafficLights.clear()
        for object in detected:
            if LabelGroups.TrafficLight in object.label.groups:
                msg = TrafficLight()
                msg.x = object.x + object.w / 2
                msg.y = object.y + object.h / 2
                msg.distance = object.distance
                # mapping between the traffic light states
                state_mapper = {
                    Labels.TrafficLightUnknown : TrafficLight.STATE_UNKNOWN,
                    Labels.TrafficLightOff : TrafficLight.STATE_OFF,
                    Labels.TrafficLightRed : TrafficLight.STATE_RED,
                    Labels.TrafficLightYellowRed : TrafficLight.STATE_YELLOW_RED,
                    Labels.TrafficLightYellow : TrafficLight.STATE_YELLOW,
                    Labels.TrafficLightGreen : TrafficLight.STATE_GREEN,
                }
                msg.state = state_mapper.get(object.label,lambda: TrafficLight.STATE_UNKNOWN)
                # Add to list
                self.trafficLights.append(msg)

    def periodic_update(self, event):
        """
        Periodically sends an update about the currently detected objects
        :param event: the timer event ( it is ignored)
        :return: None
        """
        sign_msg = TrafficSignInfo()
        sign_msg.header.stamp = rospy.Time.now()
        sign_msg.header.frame_id = 'DetectionServiceTrafficSigns'
        sign_msg.speedSigns.extend(self.speed_signs)
        sign_msg.stopMarks.extend(self.stop_marks)
        sign_msg.trafficLights.extend(self.trafficLights)
        self.traffic_sign_publisher.publish(sign_msg)


if __name__ == "__main__":
    service = DetectionService()
    rospy.spin()
