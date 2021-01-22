#!/usr/bin/env python
import math
from typing import List

import rospy
from simple_pid import PID
from std_msgs.msg import UInt8
from carla_msgs.msg import CarlaEgoVehicleControl, CarlaEgoVehicleStatus
from psaf_messages.msg import TrafficSignInfo, SpeedSign, TrafficLight
from nav_msgs.msg import Path, Odometry
import numpy as np
import simple_pid
from tf.transformations import euler_from_quaternion


class Distance_Controller:
    """
        Planner for the traffic rules
          e.g. the speed limits
    """

    def __init__(self, default_speed=50, distance=20):
        self.velocity_publisher = rospy.Publisher("/psaf/local_planner/speed_limit", UInt8, queue_size=1)

        rospy.Subscriber('/carla/ego_vehicle/odometry', Odometry, self.odometry_received)
        rospy.Subscriber("/move_base/PsafLocalPlanner/psaf_local_plan", Path, self.local_plan_received)

        self.current_limit = default_speed
        self.goal_point = None
        self.current_speed = 0.0
        self.current_orientation = None
        self.current_location = None

        self.pid = PID(1, 0.1, 0.05, setpoint=2)

    def local_plan_received(self, path: Path):
        if self.goal_point is None:
            self.goal_point = path.poses[500].pose.position
            print(f"goal point: {self.goal_point}")


    def odometry_received(self, odom: Odometry):
        self.current_location = odom.pose.pose.position
        self.current_orientation = odom.pose.pose.orientation
        self.current_speed = odom.twist.twist.linear.x

        if self.goal_point is None:
            return

        q = (self.current_orientation.x, self.current_orientation.y, self.current_orientation.z, self.current_orientation.w)
        _, _, yaw = euler_from_quaternion(q)
        orientation = math.degrees(yaw)

        target_vector = np.array([self.goal_point.x - self.current_location.x, self.goal_point.y - self.current_location.y])
        dist_target = np.linalg.norm(target_vector)
        forward_vector = np.array([math.cos(math.radians(orientation)), math.sin(math.radians(orientation))])
        c = np.clip(np.dot(forward_vector, target_vector) / dist_target, -1.0, 1.0)

        if c < 0:
            dist_target = dist_target * -1


        self.current_limit = int(self.pid(dist_target))
        self.current_limit = max(self.current_limit, 0)

        msg = UInt8(self.current_limit)
        self.velocity_publisher.publish(msg)
        print(f"distance: {dist_target} | new speed limit: {self.current_limit}")



if __name__ == '__main__':
    rospy.init_node("Distance_Controller")
    planner = Distance_Controller()
    rospy.spin()
