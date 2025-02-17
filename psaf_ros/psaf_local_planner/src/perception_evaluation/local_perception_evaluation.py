#!/usr/bin/env python
import math
from collections import Counter
from typing import List

import rospy
from carla_msgs.msg import CarlaEgoVehicleControl, CarlaEgoVehicleStatus
from psaf_messages.msg import TrafficSignInfo, TrafficLight, TrafficSituation, StopLineInfo, StopLine


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

        self.publish_timer = rospy.Timer(rospy.Duration(0.05), self.periodic_planner_input_update)

        self.last_traffic_light_states = []
        self.distance_to_stop_line = float('inf')

        self.steer = 0.

    def callback_ego_vehicle_cmd(self, data: CarlaEgoVehicleControl):
        """
        Collects the ego vehicle data as callback for the subscriber
        :param data: the message
        :return: None
        """
        self.steer = data.steer

    def look_for_traffic_light(self, traffic_lights: List[TrafficLight]):
        """
        Analyzes the given list of traffic lights and compute the current state regarding the traffic light
        :param traffic_lights: list of traffic lights
        :return: None
        """

        # Copy the value to work always with he same data
        steer = self.steer

        def filter_traffic_light(traffic_light: TrafficLight) -> bool:
            # only traffic signs that are within 100m and when the state is known
            if traffic_light.state != traffic_light.STATE_UNKNOWN and traffic_light.distance < 100:
                # check on right hand side for traffic lights (e.g. in small towns like town02)
                if traffic_light.distance < 50:
                    x1 = steer * (0.3 if steer > 0 else 0.6) + 0.5
                    # Calculate coordinates
                    x2 = min([x1 + 0.4, 1.])
                    first_check = is_in_traffic_light_area(traffic_light, x1, x2, 0.37, 0.8)
                    second_check = traffic_light.x>0.7 and traffic_light.distance < 4 # If we are very close to the traffic light
                    if first_check or second_check:
                        return True
                # check for traffic lights that are in front of the car on the other side of the intersection (american style)
                # scale width and height by distance -> size decrease with distance
                scaling = 1 - math.pow(4, -(traffic_light.distance) / 100)
                center_x = steer * (0.25 if steer > 0 else 0.3) + 0.5
                center_y = 0.375 + 0.1 * scaling
                width = 0.3 * scaling
                height = 0.25 * scaling


                return is_in_traffic_light_area(traffic_light, center_x - width / 2, center_x + width / 2,
                                                0., center_y + height / 2)

        filtered_traffic_light_list = list(filter(filter_traffic_light, traffic_lights))
        if len(filtered_traffic_light_list) > 0:
            most_common_state, _ = Counter(map(lambda x: x.state, filtered_traffic_light_list)).most_common(1)[0]
            closest_with_correct_state = next(iter(
                sorted(
                    filter(lambda x: x.state == most_common_state, filtered_traffic_light_list),
                    key=lambda x: x.distance)), None)

            self.last_traffic_light_states.append(closest_with_correct_state)
        else:
            self.last_traffic_light_states.append(None)
        if len(self.last_traffic_light_states) >= 3:
            self.last_traffic_light_states.pop(0)

    def callback_traffic_signs(self, traffic_signs: TrafficSignInfo):
        """
        Callback to handle the traffic sign data
        :param traffic_signs: the message
        :return: None
        """
        self.look_for_traffic_light(traffic_signs.trafficLights)

    def callback_stop_lines(self, stop_lines: StopLineInfo):
        """
           Callback to handle the stop line info
           :param stop_lines: the message
           :return: None
           """
        self.distance_to_stop_line = float('inf')
        if list(stop_lines.stopLines):
            self.distance_to_stop_line = min(map(lambda x: x.distance, stop_lines.stopLines))

    def periodic_planner_input_update(self, event):
        """
        Function that should be run periodically to update the local planner traffic situation topic
        :return: None
        """
        try:
            msg = TrafficSituation()
            if len(self.last_traffic_light_states) > 0:
                most_common_state, _ = Counter(map(lambda x: x.state if x is not None else None, self.last_traffic_light_states)).most_common(1)[0]
                majority_representative = next(iter(
                    sorted(
                        filter(lambda x: x.state == most_common_state if x is not None else False, self.last_traffic_light_states),
                        key=lambda x: x.distance)), None)
                if majority_representative is not None:
                    msg.trafficLight.append(majority_representative)
            msg.distanceToStopLine = self.distance_to_stop_line
            msg.header.stamp = rospy.Time.now()
            msg.header.frame_id = 'car'
            self.traffic_situation_publisher.publish(msg)
        except IndexError as e:
            rospy.logerr(f"Out of range error in local perception eval: {e}")


if __name__ == '__main__':
    rospy.init_node("Local_perception_evaluation")
    planner = LocalPerceptionEvaluation()
    rospy.spin()
