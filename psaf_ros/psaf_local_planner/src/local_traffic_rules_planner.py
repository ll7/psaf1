#!/usr/bin/env python
from typing import List

import rospy
from std_msgs.msg import UInt8
from carla_msgs.msg import CarlaEgoVehicleControl
from psaf_messages.msg import TrafficSignInfo, SpeedSign
import numpy as np

class Local_Traffic_Rules_Planner:
    """
        Planner for the traffic rules
          e.g. the speed limits
    """

    def __init__(self,default_speed= 50):
        self.velocity_publisher = rospy.Publisher("/psaf/local_planner/speed_limit", UInt8, queue_size=1)

        rospy.Subscriber("/carla/ego_vehicle/vehicle_control_cmd", CarlaEgoVehicleControl, self.callback_ego_vehicle_cmd)
        rospy.Subscriber("/psaf/perception/traffic_signs", TrafficSignInfo, self.callback_traffic_signs)

        self.publish_timer = rospy.Timer(rospy.Duration(0.1), self.periodic_planner_input_update)

        self.speed_acceptance_area = np.zeros([3, 3], dtype=float)

        # Current speed in km/h
        self.current_speed = default_speed

    def callback_ego_vehicle_cmd(self,data: CarlaEgoVehicleControl):

        width = 0.34
        x1 = data.steer * (0.3 if data.steer > 0 else 0.6) + 0.5

        # Calculate coordinates
        x2 = min([x1 + width, 1.])
        self.speed_acceptance_area = np.array([x1, 0.2, x2, 0.8], dtype=float)

    def look_for_speed_limit(self,speed_signs:List[SpeedSign]):
        edited_speed_list = sorted(filter(lambda x: x.distance < 15, speed_signs), key=lambda x: x.distance)

        area = self.speed_acceptance_area
        matches = list(
            filter(lambda sign: area[0] <= sign.x <= area[2] and area[1] <= sign.y <= area[3], edited_speed_list))

        if len(matches) > 0:
            self.current_speed = matches[0].limit

    def callback_traffic_signs(self,traffic_signs:TrafficSignInfo):
        """
        Callback to handle the traffic sign data
        :param traffic_signs: the message
        :return: None
        """
        self.look_for_speed_limit(traffic_signs.speedSigns)


    def periodic_planner_input_update(self,event):
        """
        Function that should be run periodically to update the local planner control topics
        :return: None
        """
        msg = UInt8(self.current_speed)

        self.velocity_publisher.publish(msg)
        print(f"I updated the velocity to {msg}") # TODO remove



if __name__ == '__main__':
    rospy.init_node("Local_Traffic_Rules_planner")
    planner = Local_Traffic_Rules_Planner()
    rospy.spin()


