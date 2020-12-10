#!/usr/bin/env python

import lanelet2
from lanelet2.core import GPSPoint
from lanelet2.projection import UtmProjector
from psaf_planning.global_planner.path_provider_abstract import PathProviderAbstract
from commonroad.scenario.trajectory import State
from commonroad.planning.goal import GoalRegion
import matplotlib.pyplot as plt

from commonroad.geometry.shape import Circle
from commonroad.scenario.scenario import Scenario
from commonroad.scenario.lanelet import Lanelet
from commonroad.planning.planning_problem import PlanningProblemSet, PlanningProblem
from commonroad.visualization.draw_dispatch_cr import draw_object
from commonroad.common.util import Interval, AngleInterval

from SMP.route_planner.route_planner.route_planner import RoutePlanner
import numpy as np
from SMP.route_planner.route_planner.utils_visualization import draw_route, get_plot_limits_from_reference_path, \
    get_plot_limits_from_routes
from psaf_abstraction_layer.GPS import GPS_Position, GPS_Sensor
from geometry_msgs.msg import PoseStamped, Point

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
                break
        return nearest[0]

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
        # a state with position [2.0, 3.0] m and uncertain velocity from 5.4 to 7.0 m/s
        # can be created as follows:
        start_state: State = State(position=np.array([start.x, start.y]), velocity=0,time_step=0,slip_angle=0,yaw_rate=0, orientation=0)
        goal: Lanelet = self.find_nearest_lanlet(target)
        # goal.center_vertice = [[x0, x1, ..., xn], [y0, y1, ..., yn]]
        circle = Circle(10, center=np.array([goal.center_vertices[len(goal.center_vertices)//2][0],
                                             goal.center_vertices[len(goal.center_vertices)//2][1]]))
        goal_state: State = State(position=circle,time_step=Interval(0, 10000.0) )
        goal_region: GoalRegion = GoalRegion(np.array([goal_state]), None)
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

    def _visualization(self, route):
        plot_limits = get_plot_limits_from_reference_path(route)
        # option 2: plot limits from lanelets in the route
        # plot_limits = get_plot_limits_from_routes(route)

        # determine the figure size for better visualization
        size_x = 10
        ratio_x_y = (plot_limits[1] - plot_limits[0]) / (plot_limits[3] - plot_limits[2])
        fig = plt.figure(figsize=(size_x, size_x / ratio_x_y))
        fig.gca().axis('equal')

        draw_route(route, draw_route_lanelets=True, draw_reference_path=True, plot_limits=plot_limits)

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

        # COMPUTE SHORTEST ROUTE
        # instantiate a route planner
        route_planner = RoutePlanner(self.scenario, planning_problem, backend=RoutePlanner.Backend.PRIORITY_QUEUE)

        # plan routes, and save the found routes in a route candidate holder
        candidate_holder = route_planner.plan_routes()
        # get first route in the candidate list
        route = candidate_holder.retrieve_first_route()
        self._visualization(route)
        path_poses = []
        if route is not None:
            # first clear potential previous messages
            self.path = Path()
            prev_local_point = None
            for path_point in route.reference_path:
                local_point = Point(path_point[0], path_point[1], 0)
                if prev_local_point is None:
                    prev_local_point = local_point
                path_poses.append(self._get_Pose_Stamped(local_point, prev_local_point))
                prev_local_point = local_point

            # create self.path messages
            self.path.poses = path_poses
            self.path.header.frame_id = "map"
            self.path.header.seq = 1
            self.path.header.stamp = rospy.Time.now()
            rospy.loginfo("PathProvider: Computation of feasible path done")

        else:
            # no path was found, only put start point in path
            start_point = Point(start_local_3d.x, start_local_3d.y, start_local_3d.z)
            # create self.path and self.path_long messages
            self.path.poses = [self._get_Pose_Stamped(start_point, start_point)]
            self.path.header.frame_id = "map"
            self.path.header.seq = 1
            self.path.header.stamp = rospy.Time.now()
            rospy.logerr("PathProvider: No possible path was found")
