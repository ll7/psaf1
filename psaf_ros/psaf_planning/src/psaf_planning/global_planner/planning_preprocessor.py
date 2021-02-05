#!/usr/bin/env python

# -----------------------------------------------------------------------
# This node receives the planning goal position from the RVIZ goal panel
# The goal position is relayed to the global planner in a PlanningInstruction message
# If uTurn should be considered by global planner (determined by rosparams) information about obstacles are added
# -----------------------------------------------------------

import rospy
from psaf_messages.msg import PlanningInstruction
from sensor_msgs.msg import PointCloud2, NavSatFix
import sensor_msgs.point_cloud2 as pc2


class PlanningPreprocessor:

    def __init__(self):
        self.points = []
        self.goal = None
        self.inner_sub = None
        self.outer_sub = None
        self.instruction_pub = None
        self.plan_u_turn = False

    def publish_instruction(self):
        """
        Published planning instruction message
        If uTurn has to be considered this function is called when data from both lidar sensors was received
        If uTurn doesnt have to be considered it is directly called after goal was received
        """
        # get distance to nearest obstacle in forward/left direction
        min_forward = float('inf')
        min_left = float('inf')
        if self.points:
            min_forward = sorted(self.points, key=lambda tup: tup[0])[0][0]
            min_left = sorted(self.points, key=lambda tup: tup[1])[0][1]
            self.points = []

        # build output message
        out_data = PlanningInstruction()
        out_data.goalPoint = self.goal  # relay goal position
        out_data.obstacleDistanceLeft = min_left  # distance to nearest obstacle
        out_data.obstacleDistanceForward = min_forward
        out_data.planUTurn = self.plan_u_turn  # if true, uTurn should be considered by global planner

        self.instruction_pub.publish(out_data)
        print(out_data)

    def lidar_callback_outer(self, data: PointCloud2):
        """
        Callback from outer lidar, checks if collision points are in the desired area
        Only executed once, subscriber is unregistered afterwards
        """
        print('lidar_callback_outer')
        for p in pc2.read_points(data, skip_nans=True):  # iteratre through all points in pointcloud from lidar
            # p[0]: x, p[1]: y, p[2]: z, p[3]: cos, p[4]: index, p[5]: tag
            if p[5] in [1, 2, 4, 5, 9, 10, 11, 15, 16, 17, 18, 19, 20, 21, 22]:  # check if point is of type wall, car, ...
                if (2 < p[0] < 10) and (p[1] > 1):  # only consider points on the left side and forward
                    self.points.append([abs(p[0]), abs(p[1]), p[5]])  # save point
        self.outer_sub.unregister()  # we only want data from one callback
        self.outer_sub = None
        # if data from both lidars was received, both subs are None
        # now calculate nearest obstacle and publish message
        if self.inner_sub is None and self.outer_sub is None:
            self.publish_instruction()

    def lidar_callback_inner(self, data: PointCloud2):
        """
        Callback from inner lidar, checks if collision points are in the desired area
        Only executed once, subscriber is unregistered afterwards
        """
        print('lidar_callback_inner')
        for p in pc2.read_points(data, skip_nans=True):
            if p[5] in [1, 2, 4, 5, 9, 10, 11, 15, 16, 17, 18, 19, 20, 21, 22]:
                if (2 < p[0] < 10) and (p[1] > 1):
                    self.points.append([abs(p[0]), abs(p[1]), p[5]])
        self.inner_sub.unregister()
        self.inner_sub = None
        if self.inner_sub is None and self.outer_sub is None:
            self.publish_instruction()

    def subscribe_lidar(self):
        """
        Creates Subsriber for both lidar sensors
        """
        print('checking uturn')
        # if a sub is not None, data acquisition from lidars is already in progress
        if self.inner_sub is None and self.outer_sub is None:
            self.outer_sub = rospy.Subscriber('/carla/ego_vehicle/semantic_lidar/lidar_outer/point_cloud', PointCloud2,
                                              self.lidar_callback_outer, queue_size=1)
            self.inner_sub = rospy.Subscriber('/carla/ego_vehicle/semantic_lidar/lidar_inner/point_cloud', PointCloud2,
                                              self.lidar_callback_inner, queue_size=1)

    def goal_callback(self, data: NavSatFix):
        """
        Callback from RVIZ goal panel
        """
        self.goal = data  # goal point from RVIZ
        # if uturn should be planned, acquire data from lidars
        # otherwise dont cehck lidar, just relay goal to global planner
        if self.plan_u_turn:
            self.subscribe_lidar()
        else:
            self.publish_instruction()

    def main(self):
        try:
            rospy.init_node('planning_preprocessor')

            # rosparam determining if traffic riles should be obeyed, if false uTurn is always considered
            obey_rules = rospy.get_param('/path_provider/respect_traffic_rules', False)
            # rosparam determining if uTurn should be considered even when obeying traffic rules
            always_turn = rospy.get_param('/path_provider/always_u_turn', False)
            self.plan_u_turn = obey_rules or always_turn or True
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
