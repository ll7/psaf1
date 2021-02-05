#!/usr/bin/env python

from logging import info
import rospy
import math
from geometry_msgs.msg import Twist
from ackermann_msgs.msg import AckermannDrive
from psaf_messages.msg import PlanningInstruction
from rospy.core import rospyinfo
from sensor_msgs.msg import PointCloud2, NavSatFix
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.point_cloud2 import PointField
from std_msgs.msg import Header, Bool, Float32MultiArray
from nav_msgs.msg import Odometry


class PlanningPreprocessor:

    def __init__(self):
        self.points = []
        self.goal = None
        self.inner_sub = None
        self.outer_sub = None
        self.instruction_pub = None
        self.plan_u_turn = False

    def publish_instruction(self):
        print('set distances')

        min_forward = float('inf')
        min_left = float('inf')
        if self.points:
            min_forward = sorted(self.points, key=lambda tup: tup[0])[0][0]
            min_left = sorted(self.points, key=lambda tup: tup[1])[0][1]
            self.points = []

        print('min_left: ', min_left)
        print('min_forward: ', min_forward)

        out_data = PlanningInstruction()
        out_data.goalPoint = self.goal
        out_data.obstacleDistanceLeft = min_left
        out_data.obstacleDistanceForward = min_forward
        out_data.planUTurn = self.plan_u_turn

        self.instruction_pub.publish(out_data)
        print(out_data)

    def lidar_callback_outer(self, data: PointCloud2):
        print('lidar_callback_outer')
        for p in pc2.read_points(data, skip_nans=True):
            if p[5] in [1, 2, 5, 9, 11, 15, 16, 17, 18, 19, 20, 21]:
                if (p[0] > 2) and (p[1] > 1):
                    self.points.append([abs(p[0]), abs(p[1]), p[5]])
        self.outer_sub.unregister()
        self.outer_sub = None
        if self.inner_sub is None and self.outer_sub is None:
            self.publish_instruction()

    def lidar_callback_inner(self, data: PointCloud2):
        print('lidar_callback_inner')
        for p in pc2.read_points(data, skip_nans=True):
            if p[5] in [1, 2, 5, 9, 11, 15, 16, 17, 18, 19, 20, 21]:
                if (p[0] > 2) and (p[1] > 1):
                    self.points.append([abs(p[0]), abs(p[1]), p[5]])
        self.inner_sub.unregister()
        self.inner_sub = None
        if self.inner_sub is None and self.outer_sub is None:
            self.publish_instruction()

    def check_uturn(self):
        print('checking uturn')
        if self.inner_sub is None and self.outer_sub is None:
            self.outer_sub = rospy.Subscriber('/carla/ego_vehicle/semantic_lidar/lidar_outer/point_cloud', PointCloud2,
                                              self.lidar_callback_outer, queue_size=1)
            self.inner_sub = rospy.Subscriber('/carla/ego_vehicle/semantic_lidar/lidar_inner/point_cloud', PointCloud2,
                                              self.lidar_callback_inner, queue_size=1)

    def goal_callback(self, data: NavSatFix):
        self.goal = data
        if self.plan_u_turn:
            self.check_uturn()
        else:
            self.publish_instruction()

    def main(self):
        try:
            rospy.init_node('planning_preprocessor')

            obey_rules = rospy.get_param('/path_provider/respect_traffic_rules', False)
            always_turn = rospy.get_param('/path_provider/always_u_turn', False)
            self.plan_u_turn = obey_rules or always_turn
            print('plan turn: ', self.plan_u_turn)

            rospy.Subscriber('/psaf/goal/set', NavSatFix, self.goal_callback, queue_size=1)
            self.instruction_pub = rospy.Publisher('/psaf/planning_preprocessor/planning_instruction', PlanningInstruction,
                                                   queue_size=1)

            rospy.spin()

        except rospy.ROSInterruptException:
            pass


if __name__ == '__main__':
    pre = PlanningPreprocessor()
    pre.main()
