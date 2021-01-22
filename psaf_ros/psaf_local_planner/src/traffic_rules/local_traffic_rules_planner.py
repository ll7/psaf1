#!/usr/bin/env python
import math
from typing import List

import rospy
from std_msgs.msg import UInt8, Float64
from carla_msgs.msg import CarlaEgoVehicleControl
from psaf_messages.msg import TrafficSignInfo, SpeedSign
import numpy as np


class Local_Traffic_Rules_Planner:
    """
        Planner for the traffic rules
          e.g. the speed limits
    """

    def __init__(self, default_speed=50):
        self.velocity_publisher = rospy.Publisher("/psaf/local_planner/speed_limit", UInt8, queue_size=1)

        rospy.Subscriber("/carla/ego_vehicle/vehicle_control_cmd", CarlaEgoVehicleControl,
                         self.__callback_ego_vehicle_cmd)
        rospy.Subscriber("/psaf/perception/traffic_signs", TrafficSignInfo, self.__callback_traffic_signs)
        rospy.Subscriber("/psaf/local_planner/curvature", Float64, self.__callback_curvature)

        self.publish_timer = rospy.Timer(rospy.Duration(0.1), self.periodic_planner_input_update)

        self.speed_acceptance_area = np.zeros([3, 3], dtype=float)

        # known upcoming curvature in rad
        self.curvature = 0

        # Current speed in km/h
        self.current_speed_max = default_speed

        # Default speed if no traffic limit is set
        self.default_speed = default_speed

        # Time stamp when we entered an intersection with a turn
        self.last_entering_intersection_with_turn_time_stamp = None

        # Last seen accepted speed sign = Tuple(time,limit:int)
        self.last_speed_limit = None

    def __callback_ego_vehicle_cmd(self, data: CarlaEgoVehicleControl):
        """
        Callback method to gather and react to information aboud the carla ego vehicle
        :param data:
        :return:
        """

        width = 0.34
        x1 = data.steer * (0.35 if data.steer > 0 else 0.6) + 0.5

        # Calculate coordinates
        x2 = min([x1 + width, 1.])
        # Update speed sign acceptance area
        self.speed_acceptance_area = np.array([x1, 0.2, x2, 0.8], dtype=float)

    def __callback_curvature(self, message):
        """
        Callback about the planned curvature in rads
        :param message: the float64 message
        :return:
        """
        self.curvature = float(message.data)

    def look_for_speed_limit(self, timestamp, speed_signs: List[SpeedSign]):
        edited_speed_list = sorted(filter(lambda x: x.distance < 15, speed_signs), key=lambda x: x.distance)

        area = self.speed_acceptance_area
        matches = list(
            filter(lambda sign: area[0] <= sign.x <= area[2] and area[1] <= sign.y <= area[3], edited_speed_list))

        if len(matches) > 0:
            limit = matches[0].limit
            self.last_speed_limit = (timestamp, limit)
            if self.last_entering_intersection_with_turn_time_stamp is None:  # Ensure that we are not in an intersection
                self.current_speed_max = limit

    def entering_intersection(self, timestamp, curvature):
        """
        Should be called when we enter an intersection
        :return:
        """
        self.last_entering_time_stamp = None
        if abs(curvature) > math.radians(30):
            self.last_entering_intersection_with_turn_time_stamp = timestamp
            self.last_speed_limit = None

    def leaving_intersection(self, timestamp):
        """
        Should be called when we leave an intersection
        :return:
        """
        if self.last_entering_intersection_with_turn_time_stamp is not None:
            average_time_stamp = (timestamp.to_nsec() + self.last_entering_intersection_with_turn_time_stamp.to_nsec()) / 2
            # Check if we have seen a speed limit and speed limit sign was seen after the average_time
            if self.last_speed_limit is not None and \
                    self.last_speed_limit[0].to_nsec() < average_time_stamp:  #
                self.current_speed_max = self.last_speed_limit[1]
            else:  # set the max speed to default value:
                self.current_speed_max = self.default_speed
            # Drop the time stamp because we have left the intersection
            self.last_entering_intersection_with_turn_time_stamp = None

    def __callback_traffic_signs(self, traffic_signs: TrafficSignInfo):
        """
        Callback to handle the traffic sign data
        :param traffic_signs: the message
        :return: None
        """
        self.look_for_speed_limit(traffic_signs.header.stamp, traffic_signs.speedSigns)

        # If there is an traffic light or a stop sign/ within the next x meters there will be an intersection
        valid_close_traffic_lights = list(
            filter(lambda x: x.distance < 15 and x.y < 0.3 or x.distance < 3 and x.x > 0.6,
                   traffic_signs.trafficLights))
        if len(valid_close_traffic_lights) > 0 or len(traffic_signs.stopMarks) > 0:
            self.entering_intersection(traffic_signs.header.stamp, self.curvature)

        # TODO replace this in future when we will have a state machine for the intersection situations or with global plan info
        elif self.last_entering_intersection_with_turn_time_stamp is not None:
            self.leaving_intersection(traffic_signs.header.stamp)


    def periodic_planner_input_update(self, event):
        """
        Function that should be run periodically to update the local planner control topics
        :return: None
        """
        msg = UInt8(self.current_speed_max)

        self.velocity_publisher.publish(msg)
        # print(f"I updated the velocity to {msg}")  # TODO remove


if __name__ == '__main__':
    rospy.init_node("Local_Traffic_Rules_planner")
    planner = Local_Traffic_Rules_Planner()
    rospy.spin()
