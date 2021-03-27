#!/usr/bin/env python

# -----------------------------------------------------------------------
# This node receives the planning goal position from the RVIZ goal panel
# The goal position is relayed to the global planner in a PlanningInstruction message
# If uTurn should be considered by global planner (determined by rosparams) information about obstacles are added
# -----------------------------------------------------------

import rospy
from psaf_messages.msg import PlanningInstruction
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import Pose, PoseWithCovarianceStamped
import sensor_msgs.point_cloud2 as pc2


class PlanningPreprocessor:

    def __init__(self):
        self.points = []
        self.goal = None
        self.inner_sub = None
        self.outer_sub = None
        self.instruction_pub = None
        self.plan_u_turn = False
        self.perception_area = [20, 15]  # Area we want to check to see if uTurn is possible (forward, right)
        self.min_forward = 0
        self.min_left = 0
        self.vehicle_detected = False  # if a vehicle is detected within the perception_area we don't want to do a uTurn

    def find_free_area(self):
        """
        Finds bounds of largest obstacle free area left and in front of the car
        """
        bound_x = self.perception_area[0]
        bound_y = self.perception_area[1]

        area = [0.0, 0.0]
        # slowly increase area and check if obstacles are found within
        while area[0] <= self.perception_area[0] and area[1] <= self.perception_area[1]:
            area[0] += (bound_x/100)
            area[1] += (bound_y/100)

            for point in self.points:
                # check if obstacle is within area
                if 0 < point[0] < area[0] and 0 < point[1] < area[1]:
                    # check if more left or in front of car
                    if point[0] > point[1]:
                        # if it is more in front of car, we have found forward_boundary (x)
                        bound_x = point[0] - 2.1
                        # search for nearest obstacle in left-direction that also satisfies found forward_boundary
                        # y-coordinate of that obstacle is used for left_boundary (y)
                        points_y = sorted(list(filter(lambda p: p[0] < bound_x, self.points)), key=lambda p: p[1])
                        if points_y:
                            bound_y = points_y[0][1]
                        return bound_x, bound_y
                    else:
                        # if it is more left of the car, we have found left_boundary (y)
                        bound_y = point[1] - 1.1
                        # search for nearest obstacle in forward-direction that also satisfies found left_boundary
                        # x-coordinate of that obstacle is used for forward_boundary (y)
                        points_x = sorted(list(filter(lambda p: p[1] < bound_y, self.points)), key=lambda p: p[0])
                        if points_x:
                            bound_x = points_x[0][0]
                        return bound_x, bound_y

        return bound_x, bound_y

    def publish_instruction(self):
        """
        Published planning instruction message
        If uTurn has to be considered this function is called when data from both lidar sensors was received
        If uTurn doesnt have to be considered it is directly called after goal was received
        """

        # get distance to nearest obstacle in forward/left direction
        if self.plan_u_turn and (not self.vehicle_detected):
            self.min_forward, self.min_left = self.find_free_area()

        # build output message
        out_data = PlanningInstruction()
        out_data.goalPoint = self.goal  # relay goal position
        out_data.obstacleDistanceLeft = self.min_left  # distance to nearest obstacle
        out_data.obstacleDistanceForward = self.min_forward
        out_data.planUTurn = self.plan_u_turn and (not self.vehicle_detected)  # if true, uTurn should be considered by global planner

        self.instruction_pub.publish(out_data)
        
        rospy.loginfo(f"Planning Preprocessor: uTurn-Area: %.2f x %.2f" % (self.min_left, self.min_forward))
        rospy.loginfo(f"Planning Preprocessor: Plan Turn: %r" % out_data.planUTurn)
        rospy.loginfo('Planning Preprocessor: Message sent!')

        if self.vehicle_detected:
            rospy.loginfo('Not planning uTurn because oncoming traffic was detected!')

        self.points = []
        self.vehicle_detected = False

    def lidar_callback_outer(self, data: PointCloud2):
        """
        Callback from outer lidar, checks if collision points are in the desired area
        Only executed once, subscriber is unregistered afterwards
        """
        rospy.loginfo('Planning Preprocessor: Outer Lidar Data received')
        for p in pc2.read_points(data, skip_nans=True):  # iteratre through all points in pointcloud from lidar
            # p[0]: x, p[1]: y, p[2]: z, p[3]: cos, p[4]: index, p[5]: tag
            if p[5] not in [0, 3, 6, 7, 13, 21]:  # check if point is of type wall, car, ...
                # only consider points on the left side and forward
                if (-2 < p[0] < self.perception_area[0]) and (1 < p[1] < self.perception_area[1]):
                    self.points.append([abs(p[0]+2), abs(p[1])])  # save point
                    if p[5] == 10:  # if a vehicle (tag 10)is detected within the perception_area we don't want to do a uTurn
                        self.vehicle_detected = True
                        break
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
        rospy.loginfo('Planning Preprocessor: Inner Lidar Data received')
        for p in pc2.read_points(data, skip_nans=True):
            if p[5] not in [0, 3, 6, 7, 13, 21]:
                if (-2 < p[0] < self.perception_area[0]) and (1 < p[1] < self.perception_area[1]):
                    self.points.append([abs(p[0]+2), abs(p[1])])
                    if p[5] == 10:
                        self.vehicle_detected = True
                        break
        self.inner_sub.unregister()
        self.inner_sub = None
        if self.inner_sub is None and self.outer_sub is None:
            self.publish_instruction()

    def subscribe_lidar(self):
        """
        Creates Subsriber for both lidar sensors
        """
        rospy.loginfo('Planning Preprocessor: checking uTurn area')
        # if a sub is not None, data acquisition from lidars is already in progress
        if self.inner_sub is None and self.outer_sub is None:
            self.outer_sub = rospy.Subscriber('/carla/ego_vehicle/semantic_lidar/lidar_outer/point_cloud', PointCloud2,
                                              self.lidar_callback_outer, queue_size=1)
            self.inner_sub = rospy.Subscriber('/carla/ego_vehicle/semantic_lidar/lidar_inner/point_cloud', PointCloud2,
                                              self.lidar_callback_inner, queue_size=1)

    def goal_callback(self, data: Pose):
        """
        Callback from RVIZ goal panel
        """
        self.goal = data  # goal Pose from RVIZ
        # if uturn should be planned, acquire data from lidars
        # otherwise dont check lidar, just relay goal to global planner
        if self.plan_u_turn:
            self.subscribe_lidar()
        else:
            self.publish_instruction()

    def start_callback(self, data: PoseWithCovarianceStamped):
        """
        Callback if vehicle position was changed (e.g. by Competition Manager)
        """
        try:
            goal = Pose()
            # get goal position from Paramater Server
            goal.position.x = rospy.get_param('competition/goal/position/x')
            goal.position.y = rospy.get_param('competition/goal/position/y')
            goal.position.z = rospy.get_param('competition/goal/position/z')

            goal.orientation.x = rospy.get_param('competition/goal/orientation/x', 0)
            goal.orientation.y = rospy.get_param('competition/goal/orientation/y', 0)
            goal.orientation.z = rospy.get_param('competition/goal/orientation/z', 0)
            goal.orientation.w = rospy.get_param('competition/goal/orientation/w', 0)

            # continue like goal position was received by RVIZ goal panel
            self.goal_callback(goal)
        except KeyError:
            rospy.logerr('Could not find Competition Managers Goal Position in Paramter Server')

    def main(self):
        try:
            rospy.init_node('planning_preprocessor')

            # rosparam determining if traffic rules should be obeyed, if false uTurn is always considered
            obey_rules = rospy.get_param('respect_traffic_rules', True)
            rospy.loginfo(f"Planning Preprocessor: Traffic Rules: %r" % obey_rules)

            # rosparam determining if uTurn should be considered even when obeying traffic rules
            always_turn = rospy.get_param('always_u_turn', False)
            self.plan_u_turn = (not obey_rules) or always_turn

            rospy.Subscriber('/psaf/goal/set', Pose, self.goal_callback, queue_size=1)
            rospy.Subscriber('/carla/ego_vehicle/initialpose', PoseWithCovarianceStamped, self.start_callback,
                             queue_size=1)

            self.instruction_pub = rospy.Publisher('/psaf/goal/set_instruction', PlanningInstruction,
                                                   queue_size=1)

            rospy.spin()

        except rospy.ROSInterruptException:
            pass


if __name__ == '__main__':
    pre = PlanningPreprocessor()
    pre.main()
