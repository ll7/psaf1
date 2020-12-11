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
from psaf_abstraction_layer.GPS import GPS_Position
from geometry_msgs.msg import Point

from nav_msgs.msg import Path
import rospy


class PathProviderCommonRoads(PathProviderAbstract):

    def __init__(self, init_rospy: bool = False, polling_rate: int = 1, timeout_iter: int = 10,
                 role_name: str = "ego_vehicle",
                 initial_search_radius: float = 1.0, step_size: float = 1.0, max_radius: float = 100):
        super(PathProviderCommonRoads, self).__init__(init_rospy, polling_rate, timeout_iter, role_name)
        self.radius = initial_search_radius
        self.step_size = step_size
        self.max_radius = max_radius
        self.scenario = self._load_scenario(polling_rate, timeout_iter)

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

        return scenario

    def find_nearest_lanlet(self, goal: Point) -> Lanelet:
        """
        Given a Point (x,y,z) -> find nearest lanelet
        :param goal: point to which the nearest lanelet should be searched
        :return: nearest lanelet to point goal
        """
        nearest = None
        curr_radius = self.radius
        while curr_radius < self.max_radius or nearest is not None:
            nearest = self.scenario.lanelet_network.lanelets_in_proximity(np.array([goal.x, goal.y]), curr_radius)
            if len(nearest) == 0:
                nearest = None
                curr_radius += self.step_size
            else:
                return nearest[0]
        return None

    def visualize_scenario(self, sce: Scenario, prob: PlanningProblem = None):
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
        # get nearest lanelet to start
        start: Lanelet = self.find_nearest_lanlet(start)
        # get nearest lanelet to target
        goal: Lanelet = self.find_nearest_lanlet(target)

        if start is None or goal is None:
            return None

        # create start and goal state for planning problem
        start_center_lane = [start.center_vertices[len(start.center_vertices)//2][0], start.center_vertices[
            len(start.center_vertices)//2][1]]
        start_state: State = State(position=np.array([start_center_lane[0], start_center_lane[1]]), velocity=0,
                                   time_step=0, slip_angle=0, yaw_rate=0, orientation=0)

        circle = Circle(10, center=np.array([goal.center_vertices[len(goal.center_vertices)//2][0],
                                             goal.center_vertices[len(goal.center_vertices)//2][1]]))
        goal_state: State = State(position=circle,time_step=Interval(0, 10000.0) )

        goal_region: GoalRegion = GoalRegion(np.array([goal_state]), None)

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
        plt.savefig("test"+str(num)+".png")

    def _get_shortest_route(self, routes_list: list):
        """
        get shortest route among all routes in route_list
        :param routes_list: list, which contains every generated route
        :return: shortest route
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
            self._create_path_message([self._get_Pose_Stamped(start_point, start_point)])
            rospy.logerr("PathProvider: No possible path was found")
            return

        # COMPUTE SHORTEST ROUTE
        # instantiate a route planner
        route_planner = RoutePlanner(self.scenario, planning_problem, backend=RoutePlanner.Backend.PRIORITY_QUEUE)

        # plan routes, and save the found routes in a route candidate holder
        candidate_holder = route_planner.plan_routes()
        all_routes, num_routes = candidate_holder.retrieve_all_routes()

        if debug:
            # if debug: show results in .png
            for i in range(0, num_routes):
                self._visualization(all_routes[i], i)

        path_poses = []
        if num_routes >= 1:
            route = self._get_shortest_route(all_routes)
            prev_local_point = None
            for path_point in route.reference_path:
                local_point = Point(path_point[0], path_point[1], 0)
                if prev_local_point is None:
                    prev_local_point = local_point

                path_poses.append(self._get_Pose_Stamped(local_point, prev_local_point))
                prev_local_point = local_point

            rospy.loginfo("PathProvider: Computation of feasible path done")

        else:
            # Creates a empty path message, which is by consent filled with, and only with the starting_point
            path_poses.append(self._get_Pose_Stamped(start_point, start_point))
            rospy.logerr("PathProvider: No possible path was found")

        # create self.path messages
        self._create_path_message(path_poses)
