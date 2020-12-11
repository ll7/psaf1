#!/usr/bin/env python
import asyncio

import rospy

from psaf_perception.SpeedSignDetector import SpeedSignDetector
from psaf_perception.AbstractDetector import DetectedObject
from psaf_perception.classes.TrafficSigns import SpeedSignElement
from std_msgs.msg import String
# from psaf_perception.msg import TrafficSignInfo


class DetectionService:
    """Stores the detected object from all detector"""

    def __init__(self):
        self.detectors = {}
        rospy.init_node("DetectionService")
        role_name = rospy.get_param("role_name", "ego_vehicle")
        # Add detector here
        self.detectors.update({"speed": SpeedSignDetector(role_name)})

        # Data
        # store all speed signs
        self.speed_signs = []

        # Ros components
        self.traffic_sign_publisher = rospy.Publisher("/psaf/perception/traffic_signs",String,queue_size=1) #TODO replace string with correct message type
        # Helpers
        self.publish_timer = rospy.Timer(rospy.Duration(0.1), self.periodic_update)
        self.lock = asyncio.Lock()


        # Collect detection
        self.detectors["speed"].set_on_detection_listener(self.__on_new_speed_Sign)

    def __on_new_speed_Sign(self, detected):
      #  async with self.lock:
        self.speed_signs.clear()
        for object in detected:
            x_center = object.x +object.w/2
            y_center = object.y +object.h/2
            if object.label.startswith("speed"):
                limit = 30
                if object.label == "speed_30":
                    limit = 30
                elif object.label == "speed_60":
                    limit = 60
                elif object.label == "speed_90":
                    limit = 90
                self.speed_signs.append(SpeedSignElement(x_center, y_center, limit))

    def periodic_update(self,event):
        self.traffic_sign_publisher.publish(f"speed signs: ["+",".join(map(str,self.speed_signs))+"]")


if __name__ == "__main__":
    service = DetectionService()
    rospy.spin()
