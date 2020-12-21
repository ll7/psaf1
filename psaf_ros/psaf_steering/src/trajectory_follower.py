#!/usr/bin/env python

from collections import deque

import numpy as np
from ackermann_msgs.msg import AckermannDrive
from numpy import math
import rospy
from matplotlib import pyplot as plt

from nav_msgs.msg import Path, Odometry
from std_msgs.msg import Header
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion, PointStamped
from tf.transformations import euler_from_quaternion, quaternion_from_euler

plot = False


class TrajectoryFollower(object):

    def __init__(self):
        self.frequency = 20
        self.loop_rate = rospy.Rate(self.frequency)
        self.control_cmd = AckermannDrive()
        self.target_reached = False

        self.target_speed = 50.0
        self.min_speed = 15.0

        self.current_location = None
        self.current_orientation = None
        self.current_speed = None

        self.min_distance = 3
        self.buffer_size = 100
        self.local_plan = deque(maxlen=self.buffer_size)
        self.global_plan = deque()

        self.global_plan_subscriber = rospy.Subscriber('/move_base/PsafLocalPlanner/psaf_global_plan',
                                                       Path, self.global_plan_received)

        self.global_plan_subscriber = rospy.Subscriber('/clicked_point',
                                                       PointStamped, self.test_goal_received)

        self.odometry_subscriber = rospy.Subscriber('/carla/ego_vehicle/odometry',
                                                    Odometry, self.odometry_received)

        self.local_plan_path = Path()
        self.local_plan_path.header.frame_id = 'map'
        self.local_plan_publisher = rospy.Publisher(
            "/local_plan", Path, queue_size=1)

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

    # c++: odom_helper
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
            # self.control_cmd.speed = (self.target_speed - abs(self.control_cmd.steering_angle) * (
            #         self.target_speed - self.min_speed)) / 3.6

            d, a = self.estimate_curvature(self.current_location, self.current_orientation, self.local_plan)
            rospy.loginfo('curvature: ' + str(a))
            fact = np.clip(a/100, 0, 1)
            self.control_cmd.speed = (self.target_speed - fact * (self.target_speed - self.min_speed)) / 3.6


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
                self.local_plan_path.poses.pop(0)


    def fill_point_buffer(self):
        """
        Fills the point buffer with points of the route
        """
        for _ in range(self.buffer_size - len(self.local_plan)):
            if self.global_plan:
                pos = self.global_plan.popleft()
                self.local_plan.append(pos)

                pose = PoseStamped()
                pose.header.frame_id = 'map'
                pose.header.stamp = rospy.Time.now()
                pose.pose.orientation = Quaternion(0, 0, 0, 0)
                pose.pose.position = pos
                self.local_plan_path.poses.append(pose)
            else:
                break

        self.publish_local_plan()


    def publish_local_plan(self):
        currentPose = PoseStamped()
        currentPose.header.frame_id = 'map'
        currentPose.header.stamp = rospy.Time.now()
        currentPose.pose.orientation = self.current_orientation
        currentPose.pose.position = self.current_location

        self.local_plan_path.header.frame_id = 'map'
        self.local_plan_path.header.stamp = rospy.Time.now()
        self.local_plan_path.poses.insert(0, currentPose)
        self.local_plan_publisher.publish(self.local_plan_path)
        self.local_plan_path.poses.pop(0)




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
        # angle = (a o b) / (|a|*|b|), here: |b| = 1
        forward_vector = np.array([math.cos(math.radians(orientation)), math.sin(math.radians(orientation))])
        c = np.clip(np.dot(forward_vector, target_vector) / dist_target, -1.0, 1.0)
        d_angle = math.degrees(math.acos(c))

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

        return dist_target, d_angle

    def estimate_curvature(self, current_location, current_orientation, local_plan):
        angles = []
        distances = []
        curvature = 0
        total_dist = 0

        dist_start, angle_start = self.compute_magnitude_angle(local_plan[0], current_location, current_orientation)
        angles.append(angle_start)
        distances.append(dist_start)
        total_dist += abs(dist_start)
        curvature += abs(angle_start)

        for i in range(len(local_plan) - 2):
            vector = np.array([local_plan[i+1].x - local_plan[i].x, local_plan[i+1].y - local_plan[i].y])
            dist = np.linalg.norm(vector)
            vector_next = np.array([local_plan[i + 2].x - local_plan[i + 1].x, local_plan[i + 2].y - local_plan[i + 1].y])
            dist_next = np.linalg.norm(vector_next)

            if dist == 0 or dist_next == 0:
                continue

            c = np.clip(np.dot(vector, vector_next) / (dist * dist_next), -1.0, 1.0)
            angle = math.degrees(math.acos(c))

            angles.append(angle)
            distances.append(dist)
            total_dist += abs(dist)
            curvature += abs(angle)

        return total_dist, curvature




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
