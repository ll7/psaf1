#!/usr/bin/env python

import lanelet2
from lanelet2 import traffic_rules, routing, core, geometry
from lanelet2.projection import UtmProjector
from lanelet2.core import GPSPoint
from lanelet2.io import Origin
import rospy
from psaf_planning.map_provider import MapProvider
from psaf_abstraction_layer.GPS import GPS_Position
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, Point, Quaternion
from tf.transformations import quaternion_from_euler
import math


class PathProvider:

    def __init__(self, init_rospy: bool = False, polling_rate: int = 1, timeout_iter: int = 10):
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
        self.map_provider = MapProvider()
        self.map_path = self._get_map_path(polling_rate, timeout_iter)
        if not self.map_path:
            rospy.logerr("PathProvider: No Map received!")
            self.lanelet_map = None
        else:
            self.lanelet_map = lanelet2.io.load(self.map_path, self.projector)
        self.path = Path()
        self.path_long = Path()

    def get_path_from_a_to_b(self, from_a: GPS_Position, to_b: GPS_Position):
        """
        Returns the shortest path from a to be
        :param  from_a: Start point [GPS Coord]
        :param  to_b:   End point [GPS Coord]
        :return: Pruned Path or None if no path was found at all, or no map information is received
        """
        if not self.lanelet_map:
            return self.path  # which is None, because no map was received
        self._compute_route(from_a, to_b)
        return self.path

    def get_path_from_a_to_b_long(self, from_a: GPS_Position, to_b: GPS_Position):
        """
        Returns the shortest path from a to be
        :param  from_a: Start point [GPS Coord]
        :param  to_b:   End point [GPS Coord]
        :return: Long Path or None if no path was found at all, or no map information is received
        """
        if not self.lanelet_map:
            return self.path_long  # which is None, because no map was received
        self._compute_route(from_a, to_b)
        return self.path_long

    def get_path_long(self):
        """
        Returns the current path
        :return: Long Path or None
        """
        return self.path_long

    def get_path(self):
        """
        Returns the current path
        :return: Pruned Path or None
        """
        return self.path

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

    def _euclidean_2d_distance_from_to_position(self, objA: PoseStamped, objB: lanelet2.core.BasicPoint2d):
        """
        This helper function calculates the euclidean distance between 2 Points
        :param objA: HAS! to be the path_long entry, thus a PoseStamped Object
        :param objB: BasicPoint2D provided by the lanelet package
        :return: The calculated distance
        """
        return ((objA.pose.position.x - objB.x) ** 2 + (objA.pose.position.y + objB.y) ** 2) ** 0.5

    def _get_Pose_Stamped(self, pos: Point, prev_pos: Point):
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

        if pos.x == prev_pos.x:
            euler_angle_alpha = 0.0
        else:
            euler_angle_alpha = math.tan((pos.y - prev_pos.y)/(pos.x - prev_pos.x))

        # only 2D space is relevant, therefore angles beta and gamma can be set to zero
        q = quaternion_from_euler(euler_angle_alpha, 0.0, 0.0)
        p.pose.orientation = Quaternion(*q)

        return p

    def _compute_route(self, from_a: GPS_Position, to_b: GPS_Position):
        """
        Compute shortest path
        :param from_a: Start point -- GPS Coord in float: latitude, longitude, altitude
        :param to_b: End point   -- GPS Coord in float: latitude, longitude, altitude
        """""
        rospy.loginfo("PathProvider: Computing feasible path from a to b")

        # generate traffic_rules based on participant type and location
        traffic_rules_ger = lanelet2.traffic_rules.create(lanelet2.traffic_rules.Locations.Germany,
                                                          lanelet2.traffic_rules.Participants.Vehicle)
        # generate routing_graph which represents the given map
        routing_graph = lanelet2.routing.RoutingGraph(self.lanelet_map, traffic_rules_ger)

        # step1: get nearest lanelet to start point
        gps_point_start = GPSPoint(from_a.latitude, from_a.longitude, from_a.altitude)
        start_local_3d = self.projector.forward(gps_point_start)
        start_local_2d = lanelet2.core.BasicPoint2d(start_local_3d.x, start_local_3d.y)
        close_from_lanelets = lanelet2.geometry.findNearest(self.lanelet_map.laneletLayer, start_local_2d, 2)
        from_lanelet = close_from_lanelets[0][1]

        # step2: nearest lanelet to end point
        gps_point_target = GPSPoint(to_b.latitude, to_b.longitude, to_b.altitude)
        target_local_3d = self.projector.forward(gps_point_target)
        target_local_2d = lanelet2.core.BasicPoint2d(target_local_3d.x, target_local_3d.y)
        close_to_lanelets = lanelet2.geometry.findNearest(self.lanelet_map.laneletLayer, target_local_2d, 1)
        to_lanelet = close_to_lanelets[0][1]

        # step3: compute shortest path, do not prune for path_long
        path_long_list = []
        shortest_path = routing_graph.shortestPath(from_lanelet, to_lanelet)
        prev_point = None
        if shortest_path is not None:
            # long path
            for lane in shortest_path:
                for pos in lane.centerline:
                    point = Point(pos.x, -pos.y, pos.z)
                    if prev_point is None:
                        prev_point = point
                    path_long_list.append(self._get_Pose_Stamped(point, prev_point))
                    prev_point = point

            # Prune path
            real_start_index = path_long_list.index(min(path_long_list, key=lambda
                obj_list: self._euclidean_2d_distance_from_to_position(obj_list, start_local_2d)))
            real_end_index = path_long_list.index(min(path_long_list, key=lambda
                obj_list: self._euclidean_2d_distance_from_to_position(obj_list, target_local_2d)))

            self.path.poses = path_long_list[real_start_index: real_end_index]
            self.path_long.poses = path_long_list

        else:
            # no path was found
            self.path = Path()
            self.path_long = Path()

        rospy.loginfo("PathProvider: Computation of feasible path done")


def main():
    provider = PathProvider(init_rospy=True)
    start = GPS_Position(0.001352, -0.000790, 0)  # in x,y,z: -87,9/-151,5
    end = GPS_Position(-0.001155, -0.001037, 0)  # in x,y,z: -114,5/128,6
    path = provider.get_path_from_a_to_b(start, end)
    print("Main finished")


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass