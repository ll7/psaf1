#!/usr/bin/env python

from abc import abstractmethod
import rospy
from psaf_planning.map_provider import MapProvider
from psaf_abstraction_layer.GPS import GPS_Position, GPS_Sensor
from psaf_abstraction_layer.VehicleStatus import VehicleStatusProvider
from nav_msgs.msg import Path
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import PoseStamped, Point, Quaternion
from tf.transformations import quaternion_from_euler
import math
import os
import actionlib
import pathlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import rosbag
from lanelet2.projection import UtmProjector
from lanelet2.io import Origin
import numpy as np
from std_msgs.msg import String


class PathProviderAbstract:

    def __init__(self, init_rospy: bool = False, polling_rate: int = 1, timeout_iter: int = 10,
                 role_name: str = "ego_vehicle", enable_debug: bool = False):
        """
        Class PathProvider provides a feasible path from a starting point A to a target Point B by computing
        the nearest lanelets and performing a dijkstra graph search on a provided .osm map.
        :param init_rospy: Initialize a rospy node?
        :param polling_rate: Polling Rate in [Hz]
        :param timeout_iter: Number of polling iterations until timeout occurs
        """
        self.map = None
        if init_rospy:
            # initialize node
            rospy.init_node('pathProvider', anonymous=True)
        self.origin = Origin(0, 0)
        self.projector = UtmProjector(self.origin)
        self.role_name = role_name
        self.GPS_Sensor = GPS_Sensor(role_name=self.role_name)
        self.vehicle_status = VehicleStatusProvider(role_name=self.role_name)
        rospy.Subscriber("/psaf/goal/set", NavSatFix, self._callback_goal)
        self.status_pub = rospy.Publisher('/psaf/status', String, queue_size=10)
        self.map_provider = MapProvider()
        self.path = Path()
        self.start = None
        self.start_orientation = None
        self.goal = None
        self.enable_debug = enable_debug

    def __del__(self):
        # delete path bag files
        item_list = os.listdir("/tmp")
        for item in item_list:
            if item.endswith(".path"):
                os.remove(os.path.join("/tmp", item))

    def _create_bag_files(self, filename: str, debug=False):
        """
        Creates a list of bag file objects
        :param filename: filename of bag file
        :return: list of bag file objects
        """
        bag_files = []
        # suffix set to be .path
        suffix = ".path"
        # directory
        dir_str = "/tmp/"
        bag_files.append(rosbag.Bag(str(dir_str + filename + suffix), "w"))
        # create second bag file for debugging
        if debug:
            # suffix set to be .debug
            suffix = ".debugpath"
            # debug directory, common parent folder is four directory levels above current folder
            dir_str = str(pathlib.Path(__file__).parent.absolute().parents[3]) + "/psaf_scenario/scenarios/"
            bag_files.append(rosbag.Bag(str(dir_str + filename + suffix), "w"))

        return bag_files

    @abstractmethod
    def get_path_from_a_to_b(self, from_a: GPS_Position = None, to_b: GPS_Position = None, debug=False):
        """
        Returns the shortest path from start to goal
        :param  from_a: Start point [GPS Coord] optional
        :param  to_b:   End point [GPS Coord] optional
        :param debug: Default value = False ; Generate Debug output
        :return: Path or only start if no path was found at all, or no map information is received
        """
        ...

    def _get_pose_stamped(self, pos: Point, prev_pos: Point):
        """
        This function calculates the euler angle alpha acoording to the direction vector of two given points.
        Thereafter this information is transformed to the Quaternion representation
         and the PoseStamped Object is generated
        :param pos: Position in x,y,z
        :param prev_pos: Previous Position in x,y,z
        :return: PoseStamped
        """
        p = PoseStamped()

        p.pose.position.x = pos.x
        p.pose.position.y = pos.y
        p.pose.position.z = pos.z

        # describes the relativ position of the pos to the prev pos
        rel_x = 1 if (pos.x - prev_pos.x) >= 0 else -1
        rel_y = 1 if (pos.y - prev_pos.y) >= 0 else -1

        euler_angle_yaw = math.atan2(rel_y*abs(pos.y - prev_pos.y), rel_x*abs(pos.x - prev_pos.x))

        # only 2D space is relevant, therefore angles beta and gamma can be set to zero
        q = quaternion_from_euler(0.0, 0.0, euler_angle_yaw)
        p.pose.orientation = Quaternion(*q)
        p.header.frame_id = "map"

        return p

    @abstractmethod
    def _compute_route(self, from_a: GPS_Position, to_b: GPS_Position, debug=False):
        """
        Compute shortest path
        :param from_a: Start point -- GPS Coord in float: latitude, longitude, altitude
        :param to_b: End point   -- GPS Coord in float: latitude, longitude, altitude
        :param debug: Default value = False ; Generate Debug output
        """""
        ...

    def _serialize_message(self, path: Path, debug=False):
        """
        Serialize path message and write to bag file
        """

        bag_files = self._create_bag_files(filename="path", debug=debug)
        for bag_file in bag_files:
            bag_file.write('Path', path)
            bag_file.close()

        rospy.loginfo("PathProvider: Created bag file with path. /tmp/path.path")
        if debug:
            rospy.loginfo("PathProvider: Created bag file with path. ../psaf_scenario/scenarios/")

    def _callback_goal(self, data):
        """
        Callback function of psaf goal set subscriber
        :param data: data received
        """
        self.goal = GPS_Position(latitude=data.latitude, longitude=data.longitude, altitude=data.altitude)
        self.start = self.GPS_Sensor.get_position()
        self.start_orientation = self.vehicle_status.get_status().get_orientation_as_euler()
        rospy.loginfo("PathProvider: Received start and goal position")
        self.get_path_from_a_to_b(debug=self.enable_debug)
        self._trigger_move_base(self.path.poses[-1])
        rospy.loginfo("PathProvider: global planner plugin triggered")
        self.status_pub.publish("PathProvider done")

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

    def _create_path_message(self, path_poses: list, debug: bool = False):
        """
        Creates the path message
        :param path_poses: List of PoseStamped poses
        """
        # clear potential previous messages, because it would be invalid now
        self.path = Path()
        # create self.path messages
        # pruned poses for use in rviz
        path_poses_pruned = self._prune_path_to_rviz_max_len(path_poses)
        self.path.poses = path_poses_pruned
        self.path.header.frame_id = "map"
        self.path.header.seq = 1
        self.path.header.stamp = rospy.Time.now()
        rospy.loginfo("PathProvider: Path message created")
        # then serialize message
        self._serialize_message(self.path, debug)

    def _prune_path_to_rviz_max_len(self, path_poses: list):
        """
        Prunes the path poses list to the max allowed length by rviz, which is 16384.
        Therefore poses from the path are randomly (uniformly distributed) deleted.
        Start and Target point will be kept!
        :param path_poses: List of PoseStamped poses
        :return: pruned path_poses list
        """
        # only prune if needed
        if len(path_poses) <= 16384:
            return path_poses

        max_len_rviz = 16384

        # get index list of elements without start and target index
        index_list = np.array(list(range(1, len(path_poses)-2)))

        # get index of elements which should be deleted
        index_list_to_del = np.random.choice(index_list, len(path_poses)-max_len_rviz, replace=False, p=None)

        # delete and return
        return np.delete(path_poses, index_list_to_del).tolist()
