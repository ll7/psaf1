#!/usr/bin/env python

import lanelet2
from lanelet2 import traffic_rules, routing, core, geometry
from lanelet2.projection import UtmProjector
from lanelet2.core import GPSPoint
from lanelet2.io import Origin
import rospy
from psaf_abstraction_layer.GPS import GPS_Position
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, Point
from psaf_planning.global_planner.path_provider_abstract import PathProviderAbstract


class PathProviderLanelet2(PathProviderAbstract):

    def __init__(self, init_rospy: bool = False, polling_rate: int = 1, timeout_iter: int = 10, role_name: str = "ego_vehicle"):
        """
        Class PathProvider provides a feasible path from a starting point A to a target Point B by computing
        the nearest lanelets and performing a dijkstra graph search on a provided .osm map.
        :param init_rospy: Initialize a rospy node?
        :param polling_rate: Polling Rate in [Hz]
        :param timeout_iter: Number of polling iterations until timeout occurs
        """
        super(PathProviderLanelet2, self).__init__(init_rospy, polling_rate, timeout_iter, role_name)
        self.path_long = Path()
        self.map_path = self._get_map_path(polling_rate, timeout_iter)
        if not self.map_path:
            rospy.logerr("PathProvider: No Map received!")
            self.map = None
        else:
            self.map = self._load_map(self.map_path)

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

        if not self.map:
            return self.path  # which is None, because no map was received

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

    def _create_path_long_message(self, path_poses: list):
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
        rospy.loginfo("PathProvider: Path message created")

    def _compute_route(self, from_a: GPS_Position, to_b: GPS_Position, debug=False):
        """
        Compute shortest path
        :param from_a: Start point -- GPS Coord in float: latitude, longitude, altitude
        :param to_b: End point   -- GPS Coord in float: latitude, longitude, altitude
        :param debug: Default value = False ; Generate Debug output
        """""
        rospy.loginfo("PathProvider: Computing feasible path from a to b")

        # generate traffic_rules based on participant type and location
        traffic_rules_ger = lanelet2.traffic_rules.create(lanelet2.traffic_rules.Locations.Germany,
                                                          lanelet2.traffic_rules.Participants.Vehicle)

        # generate routing_graph which represents the given map
        routing_graph = lanelet2.routing.RoutingGraph(self.map, traffic_rules_ger)
        if debug:
            lanelet2.io.write("debuggraph.osm", routing_graph.getDebugLaneletMap(0))
            rospy.loginfo("PathProvider: :DEBUG==TRUE: routing_graph written to 'debuggraph.osm'")

        # step1: get nearest lanelet to start point
        gps_point_start = GPSPoint(from_a.latitude, from_a.longitude, from_a.altitude)
        start_local_3d = self.projector.forward(gps_point_start)
        start_local_2d = lanelet2.core.BasicPoint2d(start_local_3d.x, start_local_3d.y)
        close_from_lanelets = lanelet2.geometry.findNearest(self.map.laneletLayer, start_local_2d, 1)
        from_lanelet = close_from_lanelets[0][1]

        # step2: nearest lanelet to end point
        gps_point_target = GPSPoint(to_b.latitude, to_b.longitude, to_b.altitude)
        target_local_3d = self.projector.forward(gps_point_target)
        target_local_2d = lanelet2.core.BasicPoint2d(target_local_3d.x, target_local_3d.y)
        close_to_lanelets = lanelet2.geometry.findNearest(self.map.laneletLayer, target_local_2d, 1)
        to_lanelet = close_to_lanelets[0][1]

        # step3: compute shortest path, do not prune for path_long

        path_long_list = []
        path_short_list = []
        route = routing_graph.getRoute(from_lanelet, to_lanelet, 0)

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
                    path_long_list.append(self._get_Pose_Stamped(point, prev_point))
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
            path_long_list.append(self._get_Pose_Stamped(start_point, start_point))
            path_short_list = path_long_list
            rospy.logerr("PathProvider: No possible path was found")

        # create self.path messages
        self._create_path_message(path_short_list)
        self._create_path_long_message(path_long_list)
