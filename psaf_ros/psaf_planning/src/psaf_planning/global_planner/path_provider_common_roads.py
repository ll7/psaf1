#!/usr/bin/env python

from lanelet2.core import GPSPoint
from psaf_planning.global_planner.path_provider_abstract import PathProviderAbstract
from commonroad.scenario.trajectory import State
from commonroad.planning.goal import GoalRegion
import matplotlib.pyplot as plt

from commonroad.geometry.shape import Circle
from commonroad.scenario.scenario import Scenario
from commonroad.scenario.lanelet import Lanelet
from commonroad.planning.planning_problem import PlanningProblem
from commonroad.visualization.draw_dispatch_cr import draw_object
from commonroad.common.util import Interval

from SMP.route_planner.route_planner.route_planner import RoutePlanner
import numpy as np
from SMP.route_planner.route_planner.utils_visualization import draw_route, get_plot_limits_from_reference_path
from psaf_abstraction_layer.sensors.GPS import GPS_Position
from geometry_msgs.msg import Point, PoseStamped
import sys

import rospy


class PathProviderCommonRoads(PathProviderAbstract):

    def __init__(self, init_rospy: bool = False, polling_rate: int = 1, timeout_iter: int = 10,
                 role_name: str = "ego_vehicle",
                 initial_search_radius: float = 1.0, step_size: float = 1.0,
                 max_radius: float = 100, enable_debug: bool = False):
        super(PathProviderCommonRoads, self).__init__(init_rospy, polling_rate, timeout_iter, role_name, enable_debug=enable_debug)
        self.radius = initial_search_radius
        self.step_size = step_size
        self.max_radius = max_radius
        self.map = self._load_scenario(polling_rate, timeout_iter)

    def _load_scenario(self, polling_rate: int, timeout_iter: int):
        """
        Gets the scenario of the converted .xodr map
        :param polling_rate: Polling Rate in [Hz]
        :param timeout_iter: Number of polling iterations until timeout occurs
        :return: absolute path of temporary map file
        """
        iter_cnt = 0  # counts the number of polling iterations
        r = rospy.Rate(polling_rate)  # 1Hz -> Try once a second
        scenario = None
        while not rospy.is_shutdown() and not scenario and iter_cnt < timeout_iter:
            rospy.loginfo("PathProvider: Waiting for map information")
            scenario = self.map_provider.convert_od_to_lanelet()
            r.sleep()
            iter_cnt += 1
        if not scenario:
            rospy.logerr("PathProvider: Couldn't load the map, shutting down ...")
            self.status_pub.publish("Couldn't load the map, shutting down")
            sys.exit(-1)
        return scenario

    def _find_nearest_lanlet(self, goal: Point) -> Lanelet:
        """
        Given a Point (x,y,z) -> find nearest lanelet
        :param goal: point to which the nearest lanelet should be searched
        :return: nearest lanelet to point goal
        """
        nearest = None
        curr_radius = self.radius
        while curr_radius < self.max_radius or nearest is not None:
            nearest = self.map.lanelet_network.lanelets_in_proximity(np.array([goal.x, goal.y]), curr_radius)
            if len(nearest) == 0:
                nearest = None
                curr_radius += self.step_size
            else:
                return nearest[0]
        self.status_pub.publish("Couldn't find lanlet for point" + str(goal.x)+", " + str(goal.y))
        return None

    def _visualize_scenario(self, sce: Scenario, prob: PlanningProblem = None):
        plt.figure(figsize=(10, 10))
        draw_object(sce, draw_params={'time_begin': 0})
        if prob is not None:
            draw_object(prob)
        plt.gca().set_aspect('equal')
        plt.show()

    def _generate_planning_problem(self, start: Point, target: Point) -> PlanningProblem:
        """
        Generate the planning problem by setting the starting point and generating a target region
        :param start:
        :param target:
        :return:
        """
        if self.map is None:
            # No map -> generating of planning problem not possible
            self.status_pub.publish("No map -> Planning aborted")
            return None

        # check if the car is already on a lanelet, if not get nearest lanelet to start
        start_lanelet: Lanelet
        matching_lanelet = self.map.lanelet_network.find_lanelet_by_position([np.array([start.x, start.y])])
        if len(matching_lanelet[0]) == 0:
            start_lanelet = self._find_nearest_lanlet(start)
        else:
            start_lanelet = self.map.lanelet_network.find_lanelet_by_id(matching_lanelet[0][0])
        # get nearest lanelet to target
        goal_lanelet: Lanelet = self._find_nearest_lanlet(target)

        if start_lanelet is None or goal_lanelet is None:
            return None

        # create start and goal state for planning problem
        start_position = []
        if len(matching_lanelet[0]) == 0:
            start_position = [start_lanelet.center_vertices[len(start_lanelet.center_vertices)//2][0],
                              start_lanelet.center_vertices[len(start_lanelet.center_vertices)//2][1]]
        else:
            start_position = [start.x, start.y]
        start_state: State = State(position=np.array([start_position[0], start_position[1]]), velocity=0,
                                   time_step=0, slip_angle=0, yaw_rate=0, orientation=self.start_orientation[2])

        circle = Circle(5, center=np.array([goal_lanelet.center_vertices[len(goal_lanelet.center_vertices)//2][0],
                                             goal_lanelet.center_vertices[len(goal_lanelet.center_vertices)//2][1]]))
        goal_state: State = State(position=circle, time_step=Interval(0, 10000.0))

        goal_region: GoalRegion = GoalRegion([goal_state], None)

        # return planning problem with start_state and goal_region
        return PlanningProblem(0, start_state, goal_region)

    def get_path_from_a_to_b(self, from_a: GPS_Position = None, to_b: GPS_Position = None, debug=False):
        """
        Returns the shortest path from start to goal
        :param  from_a: Start point [GPS Coord] optional
        :param  to_b:   End point [GPS Coord] optional
        :param debug: Default value = False ; Generate Debug output
        :return: Path or only start if no path was found at all, or no map information is received
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

        return self.path

    def _visualization(self, route, num):
        plot_limits = get_plot_limits_from_reference_path(route)
        # option 2: plot limits from lanelets in the route
        # plot_limits = get_plot_limits_from_routes(route)

        # determine the figure size for better visualization
        size_x = 10
        ratio_x_y = (plot_limits[1] - plot_limits[0]) / (plot_limits[3] - plot_limits[2])
        fig = plt.figure(figsize=(size_x, size_x / ratio_x_y))
        fig.gca().axis('equal')

        draw_route(route, draw_route_lanelets=True, draw_reference_path=True, plot_limits=plot_limits)
        plt.savefig("route_"+str(num)+".png")
        plt.close()

    def _euclidean_2d_distance_from_to_position(self, route_point: PoseStamped, compare_point: Point):
        """
        This helper function calculates the euclidean distance between 2 Points
        :param route_point: Has to be the point entry in path. -> PoseStamped
        :param compare_point: Point to compare the route_point -> Type Point
        :return:
        """
        return ((route_point.pose.position.x - compare_point.x) ** 2 +
                (route_point.pose.position.y - compare_point.y) ** 2) ** 0.5

    def _get_shortest_route(self, routes_list: list):
        """
        Heuristik to get the shortest route among all routes in route_list
        :param routes_list: list, which contains every generated route
        :return: shortest route determined by the number of points in the path
        """
        min_length = len(routes_list[0].reference_path)
        min_index = 0
        for index, route in enumerate(routes_list):
            if len(route.reference_path) < min_length:
                min_length = len(route.reference_path)
                min_index = index
        return routes_list[min_index]

    def _compute_route(self, from_a: GPS_Position, to_b: GPS_Position, debug=False):
        """
        Compute shortest path
        :param from_a: Start point -- GPS Coord in float: latitude, longitude, altitude
        :param to_b: End point   -- GPS Coord in float: latitude, longitude, altitude
        :param debug: Default value = False ; Generate Debug output
        """""

        # first transform GPS_Position
        gps_point_start = GPSPoint(from_a.latitude, from_a.longitude, from_a.altitude)
        start_local_3d = self.projector.forward(gps_point_start)
        start_point = Point(start_local_3d.x, start_local_3d.y, start_local_3d.z)

        gps_point_target = GPSPoint(to_b.latitude, to_b.longitude, to_b.altitude)
        target_local_3d = self.projector.forward(gps_point_target)
        target_point = Point(target_local_3d.x, target_local_3d.y, start_local_3d.z)

        # then generate planning_problem
        planning_problem = self._generate_planning_problem(start_point, target_point)

        if planning_problem is None:
            # planning_problem is None if e.g. start or goal position has not been found
            # Creates a empty path message, which is by consent filled with, and only with the starting_point
            rospy.logerr("PathProvider: Path computation aborted, insufficient information (start, target or map)")
            self._create_path_message([self._get_pose_stamped(start_point, start_point)])
            return

        # COMPUTE SHORTEST ROUTE
        # instantiate a route planner
        route_planner = RoutePlanner(self.map, planning_problem, backend=RoutePlanner.Backend.PRIORITY_QUEUE)

        # plan routes, and save the found routes in a route candidate holder
        candidate_holder = route_planner.plan_routes()
        all_routes, num_routes = candidate_holder.retrieve_all_routes()

        if debug:
            # if debug: show results in .png
            for i in range(0, num_routes):
                self._visualization(all_routes[i], i)

        rospy.loginfo("PathProvider: Computing feasible path from a to b")
        path_poses = []
        if num_routes >= 1:
            route = self._get_shortest_route(all_routes)
            prev_local_point = None
            for path_point in route.reference_path:
                local_point = Point(path_point[0], path_point[1], 0)
                if prev_local_point is None:
                    prev_local_point = local_point

                path_poses.append(self._get_pose_stamped(local_point, prev_local_point))
                prev_local_point = local_point

            # Path was found!
            # Prune path to get exact start and end points
            real_start_index = path_poses.index(min(path_poses, key=lambda
                pos: self._euclidean_2d_distance_from_to_position(pos, start_point)))
            real_end_index = path_poses.index(min(path_poses, key=lambda
                pos: self._euclidean_2d_distance_from_to_position(pos, target_point)))
            path_poses_shortened = path_poses[real_start_index:real_end_index]
            rospy.loginfo("PathProvider: Computation of feasible path done")

            # check if you are already closer to end point than to start point (rare condition)
            if len(path_poses_shortened) == 0:
                path_poses_shortened.append(path_poses[real_end_index])

            path_poses = path_poses_shortened
        else:
            # Creates a empty path message, which is by consent filled with, and only with the starting_point
            path_poses.append(self._get_pose_stamped(start_point, start_point))
            rospy.logerr("PathProvider: No possible path was found")

        # create self.path messages
        self._create_path_message(path_poses, debug)
