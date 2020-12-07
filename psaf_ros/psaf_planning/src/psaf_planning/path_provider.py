#!/usr/bin/env python

import lanelet2
from lanelet2 import traffic_rules, routing, core, geometry
from lanelet2.projection import UtmProjector
from lanelet2.core import GPSPoint
from lanelet2.io import Origin
import rospy
from psaf_planning.map_provider import MapProvider
from psaf_abstraction_layer.GPS import GPS_Position, GPS_Sensor
from nav_msgs.msg import Path
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import PoseStamped, Point, Quaternion
from tf.transformations import quaternion_from_euler
import math
import os
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import rosbag


class PathProvider:

    def __init__(self, init_rospy: bool = False, polling_rate: int = 1, timeout_iter: int = 10, role_name: str = "ego_vehicle"):
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
        self.role_name = role_name
        self.GPS_Sensor = GPS_Sensor(role_name=self.role_name)
        rospy.Subscriber("/psaf/goal/set", NavSatFix, self._callback_goal)
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
        self.start = None
        self.goal = None

    def __del__(self):
        # delete path bag files
        item_list = os.listdir("/tmp")
        for item in item_list:
            if item.endswith(".path"):
                os.remove(os.path.join("/tmp", item))

    def _create_bag_file(self, filename: str):
        """
        Creates a bag file object
        :param filename: filename of bag file
        :return: bag file object
        """
        # suffix set to be .path
        suffix = ".path"
        # directory
        dir = "/tmp/"
        return rosbag.Bag(str(dir + filename + suffix), "w")

    def get_path_from_a_to_b(self, from_a: GPS_Position = None, to_b: GPS_Position = None, debug=False, long: bool = False):
        """
        Returns the shortest path from start to goal
        :param  from_a: Start point [GPS Coord] optional
        :param  to_b:   End point [GPS Coord] optional
        :param debug: Default value = False ; Generate Debug output
        :param long: boolean weather the long path should be returned
        :return: Pruned or long Path or None if no path was found at all, or no map information is received
        """
        start = self.start
        goal = self.goal

        if not self.lanelet_map:
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
        self.lanelet_map = lanelet2.io.load(path, UtmProjector(origin))

    def _euclidean_2d_distance_from_to_position(self, objA: PoseStamped, objB: lanelet2.core.BasicPoint2d):
        """
        This helper function calculates the euclidean distance between 2 Points
        :param objA: HAS! to be the path_long entry, thus a PoseStamped Object
        :param objB: BasicPoint2D provided by the lanelet package
        :return: The calculated distance
        """
        return ((objA.pose.position.x - objB.x) ** 2 + (objA.pose.position.y - objB.y) ** 2) ** 0.5

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

        euler_angle_alpha = math.atan2(abs(pos.y - prev_pos.y), abs(pos.x - prev_pos.x))

        # only 2D space is relevant, therefore angles beta and gamma can be set to zero
        q = quaternion_from_euler(0.0, 0.0, euler_angle_alpha)
        p.pose.orientation = Quaternion(*q)

        return p

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
        routing_graph = lanelet2.routing.RoutingGraph(self.lanelet_map, traffic_rules_ger)
        if debug:
            lanelet2.io.write("debuggraph.osm", routing_graph.getDebugLaneletMap(0))

        # step1: get nearest lanelet to start point
        gps_point_start = GPSPoint(from_a.latitude, from_a.longitude, from_a.altitude)
        start_local_3d = self.projector.forward(gps_point_start)
        start_local_2d = lanelet2.core.BasicPoint2d(start_local_3d.x, start_local_3d.y)
        close_from_lanelets = lanelet2.geometry.findNearest(self.lanelet_map.laneletLayer, start_local_2d, 1)
        from_lanelet = close_from_lanelets[0][1]

        # step2: nearest lanelet to end point
        gps_point_target = GPSPoint(to_b.latitude, to_b.longitude, to_b.altitude)
        target_local_3d = self.projector.forward(gps_point_target)
        target_local_2d = lanelet2.core.BasicPoint2d(target_local_3d.x, target_local_3d.y)
        close_to_lanelets = lanelet2.geometry.findNearest(self.lanelet_map.laneletLayer, target_local_2d, 1)
        to_lanelet = close_to_lanelets[0][1]

        # step3: compute shortest path, do not prune for path_long
        path_long_list = []
        route = routing_graph.getRoute(from_lanelet, to_lanelet, 0)

        if debug:
            laneletSubmap = route.laneletSubmap()
            lanelet2.io.write("route.osm", laneletSubmap.laneletMap())

        shortest_path = route.shortestPath()
        # shortest_path = routing_graph.shortestPath(from_lanelet, to_lanelet)
        prev_point = None
        if shortest_path is not None:
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
            self.path.poses = path_long_list[real_start_index: real_end_index]
            self.path.header.frame_id = "map"
            self.path.header.seq = 1
            self.path.header.stamp = rospy.Time.now()
            # create self.path_long messages
            self.path_long.poses = path_long_list
            self.path_long.header.frame_id = "map"
            self.path_long.header.seq = 1
            self.path_long.header.stamp = rospy.Time.now()

        else:
            # no path was found
            self.path = Path()
            self.path_long = Path()

        rospy.loginfo("PathProvider: Computation of feasible path done")

    def _serialize_message(self, path: Path):
        """
        Serialize path message and write to bag file
        """
        bag_file = self._create_bag_file(filename="path")
        bag_file.write('Path', path)
        bag_file.close()

    def _callback_goal(self, data):
        self.goal = GPS_Position(latitude=data.latitude, longitude=data.longitude, altitude=data.altitude)
        self.start = self.GPS_Sensor.get_position()
        self._serialize_message(self.get_path_from_a_to_b())
        self._trigger_move_base(self.path.poses[-1])

    def _trigger_move_base(self, target: PoseStamped):
        """
        This function triggers the move_base by publishing the goal, which is later used for sanity checking
        """
        client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        client.wait_for_server()

        goal = MoveBaseGoal()
        goal.target_pose = target

        client.send_goal(goal)
        wait = client.wait_for_result()
        if not wait:
            rospy.logerr("Action server not available!")
            rospy.signal_shutdown("Action server not available!")
        else:
            rospy.loginfo(client.get_result())


def test():
    """
    Provide a simple standalone test scenario
    """
    provider = PathProvider(init_rospy=True)
    #provider._load_costum_map_path("src/psaf_planning/map.osm", Origin(49, 8))
    start = GPS_Position(0.000334, 0.000701, 0)  # in x,y,z: 78,1/-38,5
    end = GPS_Position(-0.000042, 0.000818, 0)  # in x,y,z: 90/4,7
    path = provider.get_path_from_a_to_b(start, end, debug=True)
    # get long path anyway
    long_path = provider.path_long
    provider._serialize_message(path)
    print("Main finished")

    pub = rospy.Publisher("/path", Path, queue_size=10)
    r = rospy.Rate(1)  # 1 Hz - once a second
    while not rospy.is_shutdown():
        pub.publish(path)
        r.sleep()


def main():
    provider = PathProvider(init_rospy=True)
    rospy.spin()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
