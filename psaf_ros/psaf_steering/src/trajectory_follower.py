#!/usr/bin/env python

from collections import deque

import numpy as np
from ackermann_msgs.msg import AckermannDrive
from numpy import math
import rospy
from matplotlib import pyplot as plt

from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion, Twist, PointStamped
from tf.transformations import euler_from_quaternion

plot = False


class TrajectoryFollower(object):

    def __init__(self):
        self.frequency = 20
        self.loop_rate = rospy.Rate(self.frequency)
        self.control_cmd = AckermannDrive()
        self.target_reached = False

        self.target_speed = 30.0
        self.min_speed = 10.0

        self.current_location = None
        self.current_orientation = None
        self.current_speed = None

        self.min_distance = 3
        self.buffer_size = 300
        self.local_plan = deque(maxlen=self.buffer_size)
        self.global_plan = deque()

        self.global_plan_subscriber = rospy.Subscriber('/move_base/TebLocalPlannerROS/global_plan',
                                                       Path, self.global_plan_received)

        self.global_plan_subscriber = rospy.Subscriber('/clicked_point',
                                                       PointStamped, self.test_goal_received)

        self.odometry_subscriber = rospy.Subscriber('/carla/ego_vehicle/odometry',
                                                    Odometry, self.odometry_received)

        self.twist_publisher = rospy.Publisher(
            "/carla/ego_vehicle/twist_pid", Twist, queue_size=1)

        self.akcermann_publisher = rospy.Publisher(
            "/carla/ego_vehicle/ackermann_cmd", AckermannDrive, queue_size=1)

    def test_goal_received(self, goal: PointStamped):
        self.local_plan.clear()
        self.global_plan.clear()
        self.global_plan.append(goal.point)
        rospy.loginfo("Test-Goal received")

    def global_plan_received(self, path):
        if not self.global_plan:
            for pose in path.poses:
                self.global_plan.append(pose.pose.position)
            rospy.loginfo("Global Plan received")

    def odometry_received(self, odom: Odometry):
        self.current_location = odom.pose.pose.position
        self.current_orientation = odom.pose.pose.orientation
        self.current_speed = odom.twist.twist.linear.x

    def step(self):
        self.del_old_points()

        if len(self.local_plan) < self.buffer_size:
            self.fill_point_buffer()

        if not self.local_plan and not self.global_plan:
            self.target_reached = True
            rospy.loginfo("Target reached!")
            self.control_cmd.steering_angle = 0.0
            self.control_cmd.speed = 0.0
        else:
            target_point = self.local_plan[0]
            _, angle = self.compute_magnitude_angle(target_point, self.current_location, self.current_orientation)
            self.control_cmd.steering_angle = math.radians(np.clip(angle, -60.0, 60.0))
            self.control_cmd.speed = (self.target_speed - abs(self.control_cmd.steering_angle) * (
                    self.target_speed - self.min_speed)) / 3.6

    def del_old_points(self):
        max_index = -1

        # find last overdriven point
        for i, point in enumerate(self.local_plan):
            distance, _ = self.compute_magnitude_angle(point, self.current_location, self.current_orientation)

            if distance < self.min_distance:
                max_index = i

        # delete overdriven points
        if max_index >= 0:
            for i in range(max_index + 1):
                self.local_plan.popleft()

    def fill_point_buffer(self):
        """
        Fills the point buffer with points of the route
        """
        for _ in range(self.buffer_size - len(self.local_plan)):
            if self.global_plan:
                self.local_plan.append(self.global_plan.popleft())
            else:
                break

    def compute_magnitude_angle(self, target_location, current_location, current_orientation):
        """
        Compute relative angle and distance between a target_location and a current_location
        """
        # angle of vehicle
        q = (current_orientation.x, current_orientation.y, current_orientation.z, current_orientation.w)
        _, _, yaw = euler_from_quaternion(q)
        orientation = math.degrees(yaw)

        # vector from vehicle to target point and distance
        target_vector = np.array([target_location.x - current_location.x, target_location.y - current_location.y])
        dist_target = np.linalg.norm(target_vector)

        # vector of the car and absolut angle between vehicle and target point
        forward_vector = np.array([math.cos(math.radians(orientation)), math.sin(math.radians(orientation))])
        if abs(dist_target) > 0.0:
            d_angle = math.degrees(math.acos(np.dot(forward_vector, target_vector) / dist_target))
        else:
            d_angle = 0.0

        # make angle negative or positive
        cross = np.cross(forward_vector, target_vector)
        if cross < 0:
            d_angle *= -1.0

        if plot:
            plt.clf()
            plt.xlim(-200, 200)
            plt.ylim(-200, 200)
            plt.autoscale(False)
            plt.axis([-200, 200, -200, 200])
            plt.plot(current_location.x, current_location.y, "xb")
            plt.plot(target_location.x, target_location.y, "xr")
            plt.quiver(current_location.x, current_location.y, forward_vector[0], forward_vector[1])
            plt.text(100, 100, str(d_angle), fontsize=16)
            plt.pause(0.01)
            plt.draw()

        return (dist_target, d_angle)

    def publish_ackermann(self):
        self.akcermann_publisher.publish(self.control_cmd)
        # print(self.control_cmd)

    def run(self):
        while not rospy.is_shutdown():
            self.step()
            self.publish_ackermann()
            try:
                self.loop_rate.sleep()
            except rospy.ROSInterruptException:
                pass


def main():
    rospy.init_node('trajectory_follower', anonymous=True)
    follower = TrajectoryFollower()
    try:
        follower.run()
    finally:
        del follower
        rospy.loginfo("Done")


if __name__ == "__main__":
    main()
