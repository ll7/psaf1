#!/usr/bin/env python

from psaf_ros.psaf_planning.src.map_provider import MapProvider, GPS_Position
import lanelet2
from lanelet2 import traffic_rules, routing, core, geometry
from lanelet2.projection import UtmProjector
from lanelet2.core import GPSPoint
from lanelet2.io import Origin
import rospy


class PathProvider:

    def __init__(self):
        # initialize node
        rospy.init_node('pathProvider', anonymous=True)
        self.origin = Origin(49.463075, 10.86966)
        self.projector = UtmProjector(self.origin)

        # self.map_provider = MapProvider()
        # self.map_path = self.get_map_path()
        self.lanelet_map = lanelet2.io.load("/home/tobi/psaf1/psaf_ros/psaf_planning/src/map(1).osm", self.projector)
        self.path = None

    def get_path_from_a_to_b(self, from_a, to_b):
        """
        Returns the shortest path from a to be
        :param  from_a: Start point [GPS Coord]
        :param  to_b:   End point [GPS Coord]
        :return: Path or None if no path was found at all
        """
        self.compute_route(from_a, to_b)
        return self.path

    def get_path(self):
        """
        Returns the current path
        :return: Path or None
        """
        return self.path

    def get_map_path(self):
        while not self.map_provider.map_ready:
            rospy.loginfo("Waiting for map information")
        rospy.loginfo("Map information received")
        return self.map_provider.convert_od_to_lanelet()

    def compute_route(self, from_a, to_b):
        """
        Compute shortest path
        :param from_a:  Start point -- GPS Coord in float: latitude, longitude, altitude
        :param to_b:    End point   -- GPS Coord in float: latitude, longitude, altitude
        """""
        rospy.loginfo("Computing feasible route from a to b")

        # generate traffic_rules based on participant type and location
        traffic_rules_ger = lanelet2.traffic_rules.create(lanelet2.traffic_rules.Locations.Germany,
                                                      lanelet2.traffic_rules.Participants.Vehicle)
        routing_graph = lanelet2.routing.RoutingGraph(self.lanelet_map, traffic_rules_ger)
        routing_graph.exportGraphML("graph.graphml")

        # step1: get nearest lanelet to start point
        gps_point = GPSPoint(from_a.latitude, from_a.longitude, from_a.altitude)
        p_local = self.projector.forward(gps_point)
        pp_local = lanelet2.core.BasicPoint2d(p_local.x, p_local.y)
        close_from_lanelets = lanelet2.geometry.findNearest(self.lanelet_map.laneletLayer, pp_local, 1)
        from_lanelet = close_from_lanelets[0][1]

        # step2: nearest lanelet to end point
        gps_point = GPSPoint(to_b.latitude, to_b.longitude, to_b.altitude)
        p_local = self.projector.forward(gps_point)
        pp_local = lanelet2.core.BasicPoint2d(p_local.x, p_local.y)
        close_to_lanelets = lanelet2.geometry.findNearest(self.lanelet_map.laneletLayer, pp_local, 1)
        to_lanelet = close_to_lanelets[0][1]

        # step3: compute paths from a to b
        all_paths = routing_graph.getRoute(from_lanelet, to_lanelet)
        self.path = all_paths # only testing purposes

        # step4: compute shortest path
        #self.path = []
        #for lane in all_paths.shortestPath():
        #    for pos in lane.centerline:
        #        point = self._WayPoint(pos.x, pos.y, pos.z)
        #        self.path.append(point)

    def export(self, path):
        if path is not None:
            laneletSubmap = path.laneletSubmap()
            lanelet2.io.write("route2.osm", laneletSubmap.laneletMap(), self.origin)

    class _WayPoint:

        def __init__(self, x, y, z):
            self.x = x
            self.y = y
            self.z = z


if __name__ == "__main__":
    try:
        provider = PathProvider()
        start = GPS_Position(49.46460, 10.86083, 0)
        end = GPS_Position(49.46268, 10.85927, 0)
        path = provider.get_path_from_a_to_b(start, end)
        provider.export(path)

    except rospy.ROSInterruptException:
        pass
