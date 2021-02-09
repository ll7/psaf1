#!/usr/bin/env python
import shutil
import sys
import pathlib
import os
import actionlib
import math

import numpy as np
import matplotlib.pyplot as plt

from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovariance, Pose, PoseStamped, PoseWithCovarianceStamped
from python.psaf_abstraction_layer import AckermannControl
from python.psaf_abstraction_layer import VehicleStatusProvider
from actionlib import GoalID

import rosbag
import rospy
from rosbag import ROSBagException


class ScenarioRunner:
    def __init__(self, route_file: str = "path.debugpath", init_rospy: bool = False, timeout: int = -1,
                 radius: int = 5, sample_cnt: int = 600, height: float = 10):
        if init_rospy:
            rospy.init_node('ScenarioRunner', anonymous=True)
        self.timeout: int = timeout
        self.radius = radius
        self.sample_cnt = sample_cnt
        self.route_file: str = str(pathlib.Path(__file__).parent.absolute().parents[1]) + "/scenarios/" + route_file
        if not os.path.isfile(self.route_file):
            rospy.logerr("ScenarioRunner: Scenario file doesn't exist")
            sys.exit(-1)
        self._start: PoseWithCovariance = PoseWithCovariance()
        self._goal: PoseStamped = PoseStamped()
        self.scenario_running: bool = False
        self.is_init: bool = False
        self._actual_route = list()  # list of PoseStamps
        self.planned_route = list()  # list of PoseStamps
        self.current_pose: Pose = Pose()
        self.initial_pose_publisher = rospy.Publisher('/carla/ego_vehicle/initialpose', PoseWithCovarianceStamped,
                                                      queue_size=10)
        self.cancel_movebase_publisher = rospy.Publisher('/move_base/cancel', GoalID, queue_size=10)
        self.position_subscriber = rospy.Subscriber('/carla/ego_vehicle/odometry', Odometry, self._position_listener)
        self.finish_time_stamp: float = -1.0
        self.start_time_stamp: float = -1.0
        self.height = height
        self.rate = rospy.Rate(10)  # 10hz

    def __del__(self):
        # delete all path files
        item_list = os.listdir("/tmp")
        for item in item_list:
            if item.endswith(".path"):
                os.remove(os.path.join("/tmp", item))

    def init_scenario(self):
        """
        This function initializes the ScenarioRunner, need to be run before the scenario can be executed
        :return:
        """
        self._reset_runner()
        rospy.loginfo("ScenarioRunner: Init route")
        self._unpack_route()
        self._start.pose = self.planned_route[0].pose
        self._start.pose.position.z = self.height
        self._goal = self.planned_route[len(self.planned_route) - 1]
        # Move path file to the tmp folder so that the movebase can execute the global plan
        shutil.copyfile(self.route_file, "/tmp/path.path")
        # Move vehicle to the starting line
        self._spawn_vehicle()
        rospy.loginfo("ScenarioRunner: Init finished")
        self.is_init = True

    def execute_scenario(self):
        """
        This function executes the scenario and measures it's time from start to finish
        :return:
        """
        # wait for the car to setup
        timeout_cnt = 0
        first_timeout: bool = True
        while abs(abs(self.current_pose.position.x) - abs(self.planned_route[0].pose.position.x)) > 0.0001 \
                and abs(abs(self.current_pose.position.y) - abs(self.planned_route[0].pose.position.y)) > 0.0001:
            self.rate.sleep()
            timeout_cnt += 1
            if timeout_cnt == 100 and first_timeout:
                timeout_cnt = 0
                first_timeout = False
                rospy.loginfo("ScenarioRunner: Timeout reached, respawning vehicle")
                self._spawn_vehicle()
            elif timeout_cnt == 100:
                rospy.loginfo("ScenarioRunner: execution failed")
                sys.exit(1)
        # init timers
        self.start_time_stamp = rospy.get_time()
        current = rospy.get_time()
        # start data collection
        self.scenario_running = True
        rospy.loginfo("ScenarioRunner: Starting scenario")
        self._trigger_move_base(self._goal)
        while True:
            # check if the car has reached its destiny, the goal area
            if self._euclidean_2d_distance(self._goal.pose, self.current_pose) < self.radius:
                self.finish_time_stamp = rospy.get_time()
                break
            if current - self.start_time_stamp >= self.timeout and self.timeout != -1:
                break
            if self.timeout != -1:
                current = rospy.get_time()
            else:
                current = self.start_time_stamp
        # stop data collection
        self.scenario_running = False
        self.is_init = False
        if self.finish_time_stamp > -1.0:
            rospy.loginfo("ScenarioRunner: Finished scenario successfully in {} s".format(
                str(self.finish_time_stamp - self.start_time_stamp)))
        else:
            self.finish_time_stamp = float("inf")
            rospy.loginfo("ScenarioRunner: Scenario failed")

    def plot_routes(self):
        """
        This function visualizes the actual and the planned rout for comparison
        :return:
        """
        x_planned = list()
        y_planned = list()
        x_actual = list()
        y_actual = list()
        for i in range(0, len(self.planned_route)):
            x_planned.append(self.planned_route[i].pose.position.x)
            y_planned.append(self.planned_route[i].pose.position.y)

        plt.plot(x_planned, y_planned, label="planned")

        for i in range(0, len(self._actual_route)):
            x_actual.append(self._actual_route[i].position.x)
            y_actual.append(self._actual_route[i].position.y)

        plt.plot(x_actual, y_actual, label="actual")
        plt.legend()
        plt.show()

    def evaluate_route_quality_cos(self) -> float:
        """
        This function calculates the route deviation based on the cosine law
        :return: sum off all deviations
        """
        if self.finish_time_stamp == -1.0:
            # route not finished in the given time frame
            return float("inf")
        diff = 0
        # get index list of all elements in the _actual_route list
        # (index 0 will be ignored since the car gets placed onto that point)
        index_list = np.array(list(range(1, len(self._actual_route))))
        # get index of elements which should be deleted
        if len(index_list) < self.sample_cnt:
            rospy.logerr("ScenarioRunner: The actual_route only contains {} elements, but a sample_cnt of {} was given".
                         format(str(len(self._actual_route) - 1), str(self.sample_cnt)))
            sys.exit(-1)
        index_samples = np.random.choice(index_list, self.sample_cnt, replace=False, p=None)
        for i in range(0, len(index_samples)):
            actual_pose: Pose = self._actual_route[i]

            index1, index2 = self._find_nearest_two_points(actual_pose)
            planer_poses1: Pose = self.planned_route[index1].pose
            planer_poses2: Pose = self.planned_route[index2].pose

            c = self._euclidean_2d_distance(planer_poses1, planer_poses2)
            a = self._euclidean_2d_distance(planer_poses1, actual_pose)
            b = self._euclidean_2d_distance(planer_poses2, actual_pose)

            cos_alpha = (a * a - b * b - c * c) / 2 * b * c
            diff += math.sin(math.acos(cos_alpha)) * b
        rospy.loginfo("ScenarioRunner: Global diff {}".format(str(diff)))
        return diff

    def _find_nearest_point(self, point: Pose) -> int:
        """
        This helper function determines the index of the nearest route point for the given pose
        :param point: The Pose that contains the coordinates of the reference point
        :return: index
        """
        distance: float = float("inf")
        best_index: int = 0
        for i in range(0, len(self.planned_route)):
            tmp_distance = self._euclidean_2d_distance(point, self.planned_route[i].pose)
            if tmp_distance < distance:
                distance = tmp_distance
                best_index = i
        return best_index

    def _find_nearest_two_points(self, point: Pose):
        """
        This helper function determines the indices of the two nearest route points for the given pose
        :param point: The Pose that contains the coordinates of the reference point
        :return: index1, index2
        """
        distance1: float = float("inf")
        distance2: float = float("inf")
        best_index1: int = 0
        best_index2: int = 0
        for i in range(0, len(self.planned_route)):
            tmp_distance = self._euclidean_2d_distance(point, self.planned_route[i].pose)
            if tmp_distance < distance1:
                distance1 = tmp_distance
                best_index1 = i
            elif tmp_distance < distance2:
                distance2 = tmp_distance
                best_index2 = i

        return best_index1, best_index2

    def _position_listener(self, position: Odometry):
        """
        Position callback function
        :return:
        """
        self.current_pose = position.pose.pose
        if self.scenario_running:
            self._actual_route.append(position.pose.pose)

    def _reset_runner(self):
        """
        This helper function resets all important variables
        :return:
        """
        rospy.loginfo("ScenarioRunner: Reset variables")
        while self.cancel_movebase_publisher.get_num_connections() == 0:
            self.rate.sleep()
        self.cancel_movebase_publisher.publish(GoalID())
        ack_ctrl: AckermannControl = AckermannControl()
        ack_ctrl.set_jerk(0)
        ack_ctrl.set_steering_angle(0)
        ack_ctrl.set_steering_angle_velocity(0)
        ack_ctrl.set_acceleration(0)
        ack_ctrl.set_speed(0)

        vehicle: VehicleStatusProvider = VehicleStatusProvider("ego_vehicle")
        # wait for commands to be executed
        # make sure that the car has really stopped
        while not vehicle.status_available:
            self.rate.sleep()
        current_speed = vehicle.get_status().velocity
        current_acceleration = vehicle.get_status().acceleration
        while current_speed != 0 and current_acceleration != 0:
            self.rate.sleep()
            current_speed = vehicle.get_status().velocity
            current_acceleration = vehicle.get_status().acceleration

        # reset variables
        self.finish_time_stamp: float = -1.0
        self.start_time_stamp: float = -1.0
        self.scenario_running: bool = False
        self.is_init = False
        # clear lists
        self._actual_route.clear()
        self.planned_route.clear()
        # delete all path files
        item_list = os.listdir("/tmp")
        for item in item_list:
            if item.endswith(".path"):
                os.remove(os.path.join("/tmp", item))

    def _unpack_route(self):
        """
        This function extracts the given route bag file and converts its content inits a list for further operations
        :return:
        """
        rospy.loginfo("ScenarioRunner: Starting the unpacking")
        try:
            bag = rosbag.Bag(self.route_file)
            for topic, msg, t in bag.read_messages(topics=['Path']):
                for i in range(0, len(msg.poses)):
                    pose = msg.poses[i]
                    self.planned_route.append(pose)
            bag.close()
            rospy.loginfo("ScenarioRunner: Route unpacked")
        except ROSBagException:
            rospy.logerr("ScenarioRunner: Failure while extracting the route")
            sys.exit(-1)

    def _trigger_move_base(self, target: PoseStamped):
        """
        This function triggers the move_base by publishing the last entry in path, which is later used for sanity checking
        The last entry can be the goal if a path was found or the starting point if no path was found
        """
        client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        client.wait_for_server()

        goal = MoveBaseGoal()
        goal.target_pose = target

        client.send_goal(goal)

    def _spawn_vehicle(self):
        # Move vehicle to the starting line
        spawn: PoseWithCovarianceStamped = PoseWithCovarianceStamped()
        spawn.pose = self._start
        while self.initial_pose_publisher.get_num_connections() == 0:
            self.rate.sleep()
        self.initial_pose_publisher.publish(spawn)

    def _euclidean_2d_distance(self, route_point: Pose, compare_point: Pose):
        """
        This helper function calculates the euclidean distance between 2 Points
        :param route_point: Has to be the point entry in path. -> PoseStamped
        :param compare_point: Point to compare the route_point -> Type Point
        :return:
        """
        return ((route_point.position.x - compare_point.position.x) ** 2 +
                (route_point.position.y - compare_point.position.y) ** 2) ** 0.5


def main():
    print(rospy.get_name())
    timeout = rospy.get_param('/scenario_runner_commonroad/timeout', -1.0)
    radius = rospy.get_param('/scenario_runner_commonroad/radius', 5)
    sample_cnt = rospy.get_param('/scenario_runner_commonroad/sample_cnt', 100)
    file = rospy.get_param('/scenario_runner_commonroad/file', 'path.debugpath')
    height = rospy.get_param('/scenario_runner_commonroad/height', 10)
    scenario = ScenarioRunner(init_rospy=True, timeout=timeout, radius=radius, route_file=file,
                              sample_cnt=sample_cnt, height=height)
    scenario.init_scenario()
    scenario.execute_scenario()
    scenario.evaluate_route_quality_cos()
    scenario.plot_routes()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
