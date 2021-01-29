#!/usr/bin/env python
from typing import List

import rospy
from std_msgs.msg import UInt8
from carla_msgs.msg import CarlaEgoVehicleControl, CarlaEgoVehicleStatus
from psaf_messages.msg import TrafficSignInfo, SpeedSign, TrafficLight
import numpy as np


class LocalPerceptionEvaluation:
    """
        Selects the correct data from perception to pass them to the local planner
    """

    def __init__(self):
        self.traffic_light_publisher = rospy.Publisher("/psaf/local_planner/traffic_light", TrafficLight, queue_size=1)

        rospy.Subscriber("/carla/ego_vehicle/vehicle_control_cmd", CarlaEgoVehicleControl,
                         self.callback_ego_vehicle_cmd)
        rospy.Subscriber("/psaf/perception/traffic_signs", TrafficSignInfo, self.callback_traffic_signs)

        self.publish_timer = rospy.Timer(rospy.Duration(0.1), self.periodic_planner_input_update)

        self.traffic_light_acceptance_areas = np.zeros([2, 4], dtype=float)

    def callback_ego_vehicle_cmd(self, data: CarlaEgoVehicleControl):

        width = 0.34
        x1 = data.steer * (0.3 if data.steer > 0 else 0.6) + 0.5

        # Calculate coordinates
        x2 = min([x1 + width, 1.])
        self.traffic_light_acceptance_areas[0] = np.array([x1, 0.2, x2, 0.8], dtype=float)
        self.traffic_light_acceptance_areas[1] = np.array([x1, 0.2, x2, 0.8], dtype=float)

    def is_in_traffic_light_area(self, traffic_lights: List[TrafficLight]):
        sorted_traffic_light_list = sorted(traffic_lights, key=lambda x: x.distance)
        area = self.traffic_light_acceptance_area
        matches = list(
            filter(lambda sign: area[0] <= sign.x <= area[2] and area[1] <= sign.y <= area[3],
                   sorted_traffic_light_list))

        if len(matches) > 0:
            self.next_traffic_light = matches[0]
        else:
            self.next_traffic_light = None

        if len(sorted_traffic_light_list) > 0:
            self.next_traffic_light = sorted_traffic_light_list[0]

    def look_for_traffic_light(self, traffic_lights: List[TrafficLight]):
        pass

    def callback_traffic_signs(self, traffic_signs: TrafficSignInfo):
        """
        Callback to handle the traffic sign data
        :param traffic_signs: the message
        :return: None
        """
        self.look_for_traffic_light(traffic_signs.trafficLights)

    def periodic_planner_input_update(self, event):
        """
        Function that should be run periodically to update the local planner control topics
        :return: None
        """
        # TODO
        self.traffic_light_publisher.publish(self.next_traffic_light)


if __name__ == '__main__':
    rospy.init_node("Local_Traffic_Rules_planner")
    planner = LocalPerceptionEvaluation()
    rospy.spin()
