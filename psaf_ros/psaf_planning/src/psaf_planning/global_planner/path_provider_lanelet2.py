#!/usr/bin/env python
import os

import lanelet2
import rosbag
from lanelet2 import traffic_rules, routing, core, geometry
from lanelet2.projection import UtmProjector
from lanelet2.core import GPSPoint
from lanelet2.io import Origin
import rospy
from tf.transformations import quaternion_from_euler
from geometry_msgs.msg import PoseStamped, Point, Quaternion
from python.psaf_abstraction_layer.VehicleStatus import VehicleStatusProvider
from nav_msgs.msg import Path
from sensor_msgs.msg import NavSatFix

from psaf_planning.map_provider.map_provider import MapProvider
from python.psaf_abstraction_layer.sensors.GPS import GPS_Position, GPS_Sensor
from std_msgs.msg import String
import actionlib
import pathlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import numpy as np
import math

class PathProviderLanelet2:

    def __init__(self, init_rospy: bool = False, polling_rate: int = 1, timeout_iter: int = 10,
                 role_name: str = "ego_vehicle", enable_debug: bool = False):
        """
        Class PathProvider provides a feasible path from a starting point A to a target Point B by computing
        the nearest lanelets and performing a dijkstra graph search on a provided .osm map.
        :param init_rospy: Initialize a rospy node?
        :param polling_rate: Polling Rate in [Hz]
        :param timeout_iter: Number of polling iterations until timeout occurs
        """
        if init_rospy:
            # initialize node
            rospy.init_node('pathProvider', anonymous=True)
        self.origin = Origin(0, 0)
        self.projector = UtmProjector(self.origin)
        self.role_name = role_name
        self.GPS_Sensor = GPS_Sensor(role_name=self.role_name)
        self.vehicle_status = VehicleStatusProvider(role_name=self.role_name)
        self.status_pub = rospy.Publisher('/psaf/status', String, queue_size=10)
        self.status_pub.publish("PathProvider not ready")
        self.start = None
        self.goal = None
        self.enable_debug = enable_debug
        self.map_provider = MapProvider()
        self.path_long = Path()
        self.map_path = self._get_map_path(polling_rate, timeout_iter)
        if not self.map_path:
            rospy.logerr("PathProvider: No Map received!")
            self.map = None
        else:
            self.map = self._load_map(self.map_path)
        rospy.Subscriber("/psaf/goal/set", NavSatFix, self._callback_goal)
        self.status_pub.publish("PathProvider ready")

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
    def _load_map(self, path):
        return lanelet2.io.load(path, self.projector)

    def _get_map_path(self, polling_rate: int, timeout_iter: int):
        """
        Gets the path of the converted .osm map
        :param polling_rate: Polling Rate in [Hz]
        :param timeout_iter: Number of polling iterations until timeout occurs
        :return: absolute path of temporary map file
        """
        iter_cnt = 0  # counts the number of polling iterations
        r = rospy.Rate(polling_rate)  # 1Hz -> Try once a second
        map_path = ""
        while not rospy.is_shutdown() and not map_path and iter_cnt < timeout_iter:
            rospy.loginfo("PathProvider: Waiting for map information")
            map_path = self.map_provider.convert_to_osm()
            r.sleep()
            iter_cnt += 1

        return map_path

    def _load_costum_map_path(self, path, origin):
        """
        Only used for testing puposes
        :return: Updates the map
        :param path: path to the map
        :param origin: origin of the provided map
        """
        self.map = lanelet2.io.load(path, UtmProjector(origin))

    def _euclidean_2d_distance_from_to_position(self, objA: PoseStamped, objB: lanelet2.core.BasicPoint2d):
        """
        This helper function calculates the euclidean distance between 2 Points
        :param objA: HAS! to be the path_long entry, thus a PoseStamped Object
        :param objB: BasicPoint2D provided by the lanelet package
        :return: The calculated distance
        """
        return ((objA.pose.position.x - objB.x) ** 2 + (objA.pose.position.y - objB.y) ** 2) ** 0.5

    def get_path_from_a_to_b(self, from_a: GPS_Position = None, to_b: GPS_Position = None, debug=False,
                             long: bool = False):
        """
        Returns the shortest path from start to goal
        :param  from_a: Start point [GPS Coord] optional
        :param  to_b:   End point [GPS Coord] optional
        :param debug: Default value = False ; Generate Debug output
        :param long: boolean weather the long path should be returned
        :return: Pruned or long Path or only start if no path was found at all, or no map information is received
        """
        start = self.start
        goal = self.goal

        if from_a is not None and to_b is not None:
            start = from_a
            goal = to_b

        if not debug:
            self._compute_route(start, goal)
        else:
            self._compute_route(start, goal, debug=True)

        if long:
            return self.path_long
        else:
            # return pruned path
            return self.path

    def _create_path_long_message(self, path_poses: list, debug: bool = False):
        """
        Creates the path message
        :param path_poses: List of PoseStamped poses
        """
        # clear potential previous messages, because it would be invalid now
        self.path_long = Path()
        # create self.path messages
        self.path_long.poses = path_poses
        self.path_long.header.frame_id = "map"
        self.path_long.header.seq = 1
        self.path_long.header.stamp = rospy.Time.now()
        rospy.loginfo("PathProvider: Path long message created")
        self._serialize_message(self.path, debug)

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

        euler_angle_yaw = math.atan2(rel_y * abs(pos.y - prev_pos.y), rel_x * abs(pos.x - prev_pos.x))

        # only 2D space is relevant, therefore angles beta and gamma can be set to zero
        q = quaternion_from_euler(0.0, 0.0, euler_angle_yaw)
        p.pose.orientation = Quaternion(*q)
        p.header.frame_id = "map"

        return p

    def _compute_route(self, from_a: GPS_Position, to_b: GPS_Position, debug=False):
        """
        Compute shortest path
        :param from_a: Start point -- GPS Coord in float: latitude, longitude, altitude
        :param to_b: End point   -- GPS Coord in float: latitude, longitude, altitude
        :param debug: Default value = False ; Generate Debug output
        """""

        # step1: transform GPS Data (for starting point)
        gps_point_start = GPSPoint(from_a.latitude, from_a.longitude, from_a.altitude)
        start_local_3d = self.projector.forward(gps_point_start)
        start_local_2d = lanelet2.core.BasicPoint2d(start_local_3d.x, start_local_3d.y)
        # transform GPS Data for target point
        gps_point_target = GPSPoint(to_b.latitude, to_b.longitude, to_b.altitude)
        target_local_3d = self.projector.forward(gps_point_target)
        target_local_2d = lanelet2.core.BasicPoint2d(target_local_3d.x, target_local_3d.y)

        if self.map is None:
            # Creates a empty path message, which is by consent filled with, and only with the starting_point
            start_point = Point(start_local_3d.x, start_local_3d.y, start_local_3d.z)
            rospy.logerr("PathProvider: Path computation aborted, no map available")
            self._create_path_long_message([self._get_pose_stamped(start_point, start_point)])
            self._create_path_message([self._get_pose_stamped(start_point, start_point)])
            return

        # step2: get nearest lanelets to start and end point
        close_from_lanelets = lanelet2.geometry.findNearest(self.map.laneletLayer, start_local_2d, 1)
        from_lanelet = close_from_lanelets[0][1]
        close_to_lanelets = lanelet2.geometry.findNearest(self.map.laneletLayer, target_local_2d, 1)
        to_lanelet = close_to_lanelets[0][1]

        # generate traffic_rules based on participant type and location
        traffic_rules_ger = lanelet2.traffic_rules.create(lanelet2.traffic_rules.Locations.Germany,
                                                          lanelet2.traffic_rules.Participants.Vehicle)

        # generate routing_graph which represents the given map
        routing_graph = lanelet2.routing.RoutingGraph(self.map, traffic_rules_ger)
        if debug:
            lanelet2.io.write("debuggraph.osm", routing_graph.getDebugLaneletMap(0))
            rospy.loginfo("PathProvider: :DEBUG==TRUE: routing_graph written to 'debuggraph.osm'")

        # step3: compute shortest path, do not prune for path_long

        path_long_list = []
        path_short_list = []
        route = routing_graph.getRoute(from_lanelet, to_lanelet, 0)
        rospy.loginfo("PathProvider: Computing feasible path from a to b")
        if debug:
            laneletSubmap = route.laneletSubmap()
            lanelet2.io.write("route.osm", laneletSubmap.laneletMap())
            rospy.loginfo("PathProvider: :DEBUG==TRUE: route written to 'route.osm'")

        if route is not None:
            shortest_path = route.shortestPath()
            prev_point = None

            # long path
            for lane in shortest_path:
                for pos in lane.centerline:
                    point = Point(pos.x, pos.y, pos.z)
                    if prev_point is None:
                        prev_point = point
                    path_long_list.append(self._get_pose_stamped(point, prev_point))
                    prev_point = point

            # Path was found!
            # Prune path
            real_start_index = path_long_list.index(min(path_long_list, key=lambda
                obj_list: self._euclidean_2d_distance_from_to_position(obj_list, start_local_2d)))
            real_end_index = path_long_list.index(min(path_long_list, key=lambda
                obj_list: self._euclidean_2d_distance_from_to_position(obj_list, target_local_2d)))

            # create self.path messages
            path_short_list = path_long_list[real_start_index: real_end_index]
            rospy.loginfo("PathProvider: Computation of feasible path done")

        else:
            # Creates a empty path message, which is by consent filled with, and only with the starting_point
            start_point = Point(start_local_3d.x, start_local_3d.y, start_local_3d.z)
            path_long_list.append(self._get_pose_stamped(start_point, start_point))
            path_short_list = path_long_list
            rospy.logerr("PathProvider: No possible path was found")

        # create self.path messages
        self._create_path_message(path_short_list, debug)
        self._create_path_long_message(path_long_list, debug)

    def _callback_goal(self, data):
        """
        Callback function of psaf goal set subscriber
        :param data: data received
        """
        self._reset_map()

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
        index_list = np.array(list(range(1, len(path_poses) - 2)))

        # get index of elements which should be deleted
        index_list_to_del = np.random.choice(index_list, len(path_poses) - max_len_rviz, replace=False, p=None)

        # delete and return
        return np.delete(path_poses, index_list_to_del).tolist()

    def _reset_map(self):
        pass


def main():
    provider: PathProviderLanelet2 = PathProviderLanelet2(init_rospy=True, enable_debug=False)
    rospy.spin()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass