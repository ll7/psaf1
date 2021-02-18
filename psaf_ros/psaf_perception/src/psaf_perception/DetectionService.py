#!/usr/bin/env python
from typing import List

import rospy
from psaf_messages.msg import TrafficSignInfo, SpeedSign, StopMark, StopSign, TrafficLight, StopLineInfo, StopLine

from psaf_perception.detectors.AbstractDetector import DetectedObject, Labels, LabelGroups
from psaf_perception.detectors.StopLineDetector import StopLineDetector
from psaf_perception.detectors.TrafficLightDetector import TrafficLightDetector

ROOT_TOPIC = "/psaf/perception/"


class DetectionService:
    """Stores the detected object from all detectors"""

    def __init__(self):
        self.detectors = {}
        rospy.init_node("DetectionService")
        role_name = rospy.get_param("role_name", "ego_vehicle")
        use_gpu = rospy.get_param("use_gpu", True)
        activate_traffic_light_detector = rospy.get_param("activate_traffic_light_detector", True)
        # Add detectors here
        # self.detectors.update({"trafficSign": TrafficSignDetector(role_name=role_name, use_gpu=use_gpu)})
        # self.detectors.update({"stopMarking": StopMarkDetector(role_name=role_name, use_gpu=use_gpu)})
        if activate_traffic_light_detector:
            self.detectors.update({"trafficLight": TrafficLightDetector(role_name=role_name, use_gpu=use_gpu)})
        self.detectors.update({"stopLines": StopLineDetector(role_name=role_name)})

        # Data
        # store all speed signs
        self.speed_signs: List[SpeedSign] = []
        # store all stop signs
        self.stop_signs: List[StopSign] = []
        # store all stop marks
        self.stop_marks: List[StopMark] = []
        # store all traffic lights
        self.trafficLights: List[TrafficLight] = []
        # store all stop lines
        self.stop_lines: List[StopLine] = []

        # Ros components
        self.traffic_sign_publisher = rospy.Publisher(ROOT_TOPIC + "traffic_signs", TrafficSignInfo, queue_size=1)
        self.stop_line_publisher = rospy.Publisher(ROOT_TOPIC + "stop_lines", StopLineInfo, queue_size=1)
        # Helpers
        self.publish_timer = rospy.Timer(rospy.Duration(0.05), self.periodic_update)

        # mapping between the traffic light states
        self.traffic_light_state_mapper = {
            Labels.TrafficLightUnknown: TrafficLight.STATE_UNKNOWN,
            Labels.TrafficLightRed: TrafficLight.STATE_RED,
            Labels.TrafficLightYellow: TrafficLight.STATE_YELLOW,
            Labels.TrafficLightGreen: TrafficLight.STATE_GREEN,
        }

        # Collect detection
        # self.detectors["trafficSign"].set_on_detection_listener(self.__on_new_traffic_sign)
        # self.detectors["stopMarking"].set_on_detection_listener(self.__on_new_stop)
        if activate_traffic_light_detector:
            self.detectors["trafficLight"].set_on_detection_listener(self.__on_new_traffic_light)
        self.detectors["stopLines"].set_on_detection_listener(self.__on_new_stop_lines)

    def __on_new_traffic_sign(self, _, detected: List[DetectedObject]):
        self.speed_signs.clear()
        self.stop_signs.clear()
        for each in detected:
            if LabelGroups.Speed in each.label.groups:
                limit = 30
                if each.label == Labels.Speed30:
                    limit = 30
                elif each.label == Labels.Speed60:
                    limit = 60
                elif each.label == Labels.Speed90:
                    limit = 90
                elif each.label == Labels.SpeedLimit30:
                    limit = 30
                elif each.label == Labels.SpeedLimit40:
                    limit = 40
                elif each.label == Labels.SpeedLimit60:
                    limit = 60

                msg = SpeedSign()
                msg.x = each.x + each.w / 2
                msg.y = each.y + each.h / 2
                msg.distance = each.distance
                msg.limit = int(limit)
                self.speed_signs.append(msg)

            elif each.label == Labels.StopSign:
                msg = StopSign()
                msg.x = each.x + each.w / 2
                msg.y = each.y + each.h / 2
                msg.distance = each.distance
                self.stop_signs.append(msg)

    def __on_new_stop(self, _, detected: List[DetectedObject]):
        self.stop_marks.clear()
        for each in detected:
            if each.label == Labels.StopSurfaceMarking:
                msg = StopMark()
                msg.x = each.x + each.w / 2
                msg.y = each.y + each.h / 2
                self.stop_marks.append(msg)

    def __on_new_traffic_light(self, _, detected: List[DetectedObject]):
        self.trafficLights.clear()
        for each in detected:
            if LabelGroups.TrafficLight in each.label.groups:
                msg = TrafficLight()
                msg.x = each.x + each.w / 2
                msg.y = each.y + each.h / 2
                msg.distance = each.distance
                msg.state = self.traffic_light_state_mapper.get(each.label, lambda: TrafficLight.STATE_UNKNOWN)
                # Add to list
                self.trafficLights.append(msg)

    def __on_new_stop_lines(self, _, detected: List[DetectedObject]):
        self.stop_lines.clear()
        for each in detected:
            if each.label == Labels.StopLine:
                msg = StopLine()
                msg.x = each.x + each.w / 2
                msg.y = each.y + each.h / 2
                msg.distance = each.distance
                # Add to list
                self.stop_lines.append(msg)

    def periodic_update(self, _):
        """
        Periodically sends an update about the currently detected objects
        :param event: the timer event ( it is ignored)
        :return: None
        """
        # Publish traffic sign info
        sign_msg = TrafficSignInfo()
        sign_msg.header.stamp = rospy.Time.now()
        sign_msg.header.frame_id = 'DetectionServiceTrafficSigns'
        sign_msg.speedSigns.extend(self.speed_signs)
        sign_msg.stopSigns.extend(self.stop_signs)
        sign_msg.stopMarks.extend(self.stop_marks)
        sign_msg.trafficLights.extend(self.trafficLights)
        self.traffic_sign_publisher.publish(sign_msg)
        # Publish stop line info
        stop_msg = StopLineInfo()
        stop_msg.header.stamp = rospy.Time.now()
        stop_msg.header.frame_id = 'DetectionServiceStopLines'
        stop_msg.stopLines.extend(self.stop_lines)
        self.stop_line_publisher.publish(stop_msg)


if __name__ == "__main__":
    # Start the detection service
    service = DetectionService()
    rospy.spin()
