#!/usr/bin/env python
from typing import List

import rospy
from std_msgs.msg import UInt8
from carla_msgs.msg import CarlaEgoVehicleControl, CarlaEgoVehicleStatus
from psaf_messages.msg import TrafficSignInfo, SpeedSign, TrafficLight
import numpy as np


class Local_Traffic_Rules_Planner:
    """
        Planner for the traffic rules
          e.g. the speed limits
    """

    def __init__(self, default_speed=50):
        self.velocity_publisher = rospy.Publisher("/psaf/local_planner/speed_limit", UInt8, queue_size=1)
        self.traffic_light_publisher = rospy.Publisher("/psaf/local_planner/traffic_light", TrafficLight, queue_size=1)


        rospy.Subscriber("/carla/ego_vehicle/vehicle_control_cmd", CarlaEgoVehicleControl,
                         self.callback_ego_vehicle_cmd)
        rospy.Subscriber("/psaf/perception/traffic_signs", TrafficSignInfo, self.callback_traffic_signs)
        rospy.Subscriber("/carla/ego_vehicle/vehicle_status", CarlaEgoVehicleStatus, self.callback_vehicle_status)

        self.publish_timer = rospy.Timer(rospy.Duration(0.1), self.periodic_planner_input_update)

        self.speed_acceptance_area = np.zeros([3, 3], dtype=float)
        self.traffic_light_acceptance_area = np.zeros([3, 3], dtype=float)

        self.current_limit = default_speed
        self.next_traffic_light = None
        self.navigate = True
        self.old_limit = default_speed

        # Current speed in km/h
        self.current_speed = 0.0

    def callback_ego_vehicle_cmd(self, data: CarlaEgoVehicleControl):

        width = 0.34
        x1 = data.steer * (0.3 if data.steer > 0 else 0.6) + 0.5

        # Calculate coordinates
        x2 = min([x1 + width, 1.])
        self.speed_acceptance_area = np.array([x1, 0.2, x2, 0.8], dtype=float)

    def callback_vehicle_status(self, data: CarlaEgoVehicleStatus):
        self.current_speed = data.velocity * (-1 if data.control.reverse else 1) * 3.6


    def look_for_speed_limit(self, speed_signs: List[SpeedSign]):
        edited_speed_list = sorted(filter(lambda x: x.distance < 15, speed_signs), key=lambda x: x.distance)

        area = self.speed_acceptance_area
        matches = list(
            filter(lambda sign: area[0] <= sign.x <= area[2] and area[1] <= sign.y <= area[3], edited_speed_list))

        if len(matches) > 0:
            self.current_limit = matches[0].limit

    def look_for_traffic_light(self, traffic_lights: List[TrafficLight]):
        sorted_traffic_light_list = sorted(traffic_lights, key=lambda x: x.distance)

        self.traffic_light_acceptance_area = np.array([0.2, 0.2, 0.8, 0.5], dtype=float)

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

        if self.next_traffic_light is not None and self.next_traffic_light.state == 2:
            if self.navigate:
                self.navigate = False
                self.old_limit = self.current_limit
            distance = self.next_traffic_light.distance
            print(f'red traffic light in {distance} m  | current speed is {self.current_speed}')

            if 150 >= distance > 130 and self.current_speed > 80:
                self.current_limit = 80
            if 130 >= distance > 107 and self.current_speed > 70:
                self.current_limit = 70
            if 107 >= distance > 88 and self.current_speed > 60:
                self.current_limit = 60
            if 88 >= distance > 70 and self.current_speed > 50:
                self.current_limit = 50
            if 70 >= distance > 54 and self.current_speed > 40:
                self.current_limit = 40
            if 54 >= distance > 40 and self.current_speed > 30:
                self.current_limit = 30
            if 40 >= distance > 30 and self.current_speed > 20:
                self.current_limit = 20
            if 30 >= distance > 20 and self.current_speed > 10:
                self.current_limit = 10
            if 20 >= distance > 0:
                self.current_limit = 0


        else:
            self.navigate = True
            self.current_limit = self.old_limit



    def callback_traffic_signs(self, traffic_signs: TrafficSignInfo):
        """
        Callback to handle the traffic sign data
        :param traffic_signs: the message
        :return: None
        """
        self.look_for_traffic_light(traffic_signs.trafficLights)
        if self.navigate:
            self.look_for_speed_limit(traffic_signs.speedSigns)


    def periodic_planner_input_update(self, event):
        """
        Function that should be run periodically to update the local planner control topics
        :return: None
        """
        msg = UInt8(self.current_limit)
        self.velocity_publisher.publish(msg)
        print(f"I updated the velocity to {msg}")  # TODO remove

        #self.traffic_light_publisher.publish(self.next_traffic_light)


if __name__ == '__main__':
    rospy.init_node("Local_Traffic_Rules_planner")
    planner = Local_Traffic_Rules_Planner()
    rospy.spin()
