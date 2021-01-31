#!/usr/bin/env python
from collections import Counter
from typing import List

import rospy
from carla_msgs.msg import CarlaEgoVehicleControl, CarlaEgoVehicleStatus
from psaf_messages.msg import TrafficSignInfo, TrafficLight, TrafficSituation, StopLineInfo, StopLine
import numpy as np


def is_in_traffic_light_area(traffic_light: TrafficLight, x1, x2, y1, y2):
    return x1 <= traffic_light.x <= x2 and y1 <= traffic_light.y <= y2


class LocalPerceptionEvaluation:
    """
        Selects the correct data from perception to pass them to the local planner
    """

    def __init__(self):
        self.traffic_situation_publisher = rospy.Publisher("/psaf/local_planner/traffic_situation", TrafficSituation,
                                                           queue_size=1)

        rospy.Subscriber("/carla/ego_vehicle/vehicle_control_cmd", CarlaEgoVehicleControl,
                         self.callback_ego_vehicle_cmd)
        rospy.Subscriber("/psaf/perception/traffic_signs", TrafficSignInfo, self.callback_traffic_signs)
        rospy.Subscriber("/psaf/perception/stop_lines", StopLineInfo, self.callback_stop_lines)

        self.publish_timer = rospy.Timer(rospy.Duration(0.1), self.periodic_planner_input_update)

        self.current_traffic_light = None
        self.distance_to_stop_line = float('inf')

        self.steer = 0.

    def callback_ego_vehicle_cmd(self, data: CarlaEgoVehicleControl):
        self.steer = data.steer


    def look_for_traffic_light(self,stamp, traffic_lights: List[TrafficLight]):

        # Copy the value to work always with he same data
        steer = self.steer

        def filter_traffic_light(traffic_light: TrafficLight) -> bool:
            # only traffic signs that are within 100m and when the state is known
            if traffic_light.state != traffic_light.STATE_UNKNOWN and traffic_light.distance < 100:
                # check on right hand side for traffic lights (e.g. in small towns like town02)
                x1 = steer * (0.3 if steer > 0 else 0.6) + 0.5
                # Calculate coordinates
                x2 = min([x1 + 0.34, 1.])
                first_check = is_in_traffic_light_area(traffic_light, x1, x2, 0.2, 0.8)
                if first_check:
                    return True
                # check for traffic lights that are in front of the car on the other side of the intersection (american style)
                center_x = steer * (0.25 if steer > 0 else 0.3) + 0.5
                center_y = 0.375


                # scale width and height by distance -> size decrease with distance
                scaling = 1-(traffic_light.distance-40)/100
                width = 0.2 * scaling
                height = 0.25 *scaling
                return is_in_traffic_light_area(traffic_light,center_x-width/2,center_x+width/2,center_y-height/2,center_y+height/2)

        filtered_traffic_light_list = list(filter(filter_traffic_light,traffic_lights))
        if len(filtered_traffic_light_list)>0:
            most_common_state = Counter(map(lambda x:x.state,filtered_traffic_light_list)).most_common(1)
            closest_with_correct_state = filter(lambda x:x.state== most_common_state,filtered_traffic_light_list)

            self.current_traffic_light = closest_with_correct_state
        else:
            self.current_traffic_light = None


    def callback_traffic_signs(self, traffic_signs: TrafficSignInfo):
        """
        Callback to handle the traffic sign data
        :param traffic_signs: the message
        :return: None
        """
        self.look_for_traffic_light(traffic_signs.header.stamp,traffic_signs.trafficLights)

    def callback_stop_lines(self, stop_lines: StopLineInfo):
        """
           Callback to handle the traffic sign data
           :param stop_lines: the message
           :return: None
           """
        self.distance_to_stop_line = float('inf')
        if list(stop_lines.stopLines):
             self.distance_to_stop_line = min(map(lambda x:x.distance,stop_lines.stopLines))

    def periodic_planner_input_update(self, event):
        """
        Function that should be run periodically to update the local planner control topics
        :return: None
        """
        msg = TrafficSituation()
        if self.current_traffic_light is not None:
            msg.trafficLight.append(self.current_traffic_light)
        msg.distanceToStopLine=self.distance_to_stop_line
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = 'car'
        self.traffic_situation_publisher.publish(msg)


if __name__ == '__main__':
    rospy.init_node("Local_Traffic_Rules_planner")
    planner = LocalPerceptionEvaluation()
    rospy.spin()
