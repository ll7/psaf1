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
from SMP.route_planner.route_planner.route import Route
import numpy as np
from SMP.route_planner.route_planner.utils_visualization import draw_route, get_plot_limits_from_reference_path
from psaf_abstraction_layer.sensors.GPS import GPS_Position
from psaf_planning.common_road_manager import CommonRoadManager
from geometry_msgs.msg import Point
import sys
from copy import deepcopy
import rospy
from sensor_msgs.msg import NavSatFix

from enum import Enum
from psaf_messages.msg import XRoute


class ProblemStatus(Enum):
    Success = 1
    BadTarget = 2
    BadStart = 3
    BadLanelet = 4
    BadMap = 5


class PathProviderCommonRoads(PathProviderAbstract):

    def __init__(self, init_rospy: bool = False, polling_rate: int = 1, timeout_iter: int = 10,
                 role_name: str = "ego_vehicle",
                 initial_search_radius: float = 1.0, step_size: float = 1.0,
                 max_radius: float = 100, enable_debug: bool = False, cost_traffic_light: int = 30):
        super(PathProviderCommonRoads, self).__init__(init_rospy, polling_rate, timeout_iter, role_name,
                                                      enable_debug=enable_debug)
        self.radius = initial_search_radius
        self.step_size = step_size
        self.max_radius = max_radius
        self.cost_traffic_light = cost_traffic_light
        self.path_poses = []
        self.planning_problem = None
        self.manager = None
        self.manager = CommonRoadManager(self._load_scenario(polling_rate, timeout_iter))
        self.route_id = 0
        rospy.Subscriber("/psaf/goal/set", NavSatFix, self._callback_goal)
        self.status_pub.publish("PathProvider ready")

    def _load_scenario(self, polling_rate: int, timeout_iter: int):
        """
        Gets the scenario of the converted .xodr map
        :param polling_rate: Polling Rate in [Hz]
        :param timeout_iter: Number of polling iterations until timeout occurs
        :return: loaded scenario file
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
            nearest = self.manager.map.lanelet_network.lanelets_in_proximity(np.array([goal.x, goal.y]), curr_radius)
            if len(nearest) == 0:
                nearest = None
                curr_radius += self.step_size
            else:
                return nearest[0]
        self.status_pub.publish("Couldn't find lanlet for point" + str(goal.x) + ", " + str(goal.y))
        return None

    def _visualize_scenario(self, sce: Scenario, prob: PlanningProblem = None):
        plt.figure(figsize=(10, 10))
        draw_object(sce, draw_params={'time_begin': 0})
        if prob is not None:
            draw_object(prob)
        plt.gca().set_aspect('equal')
        plt.show()

    def _generate_planning_problem(self, start: Point, target: Point) -> ProblemStatus:
        """
        Generate the planning problem by setting the starting point and generating a target region
        :param start:  x,y,z coordinates of the current staring position
        :param target: x,y,z coordinates of the current goal position
        :return: status of the generated problem
        """
        if self.manager.map is None:
            # No map -> generating of planning problem not possible
            self.status_pub.publish("No map -> Planning aborted")
            self.planning_problem = None
            return ProblemStatus.BadMap

        # check if the car is already on a lanelet, if not get nearest lanelet to start
        start_lanelet: Lanelet
        matching_lanelet = self.manager.map.lanelet_network.find_lanelet_by_position([np.array([start.x, start.y])])
        if len(matching_lanelet[0]) == 0:
            start_lanelet = self._find_nearest_lanlet(start)
        else:
            start_lanelet = self.manager.map.lanelet_network.find_lanelet_by_id(matching_lanelet[0][0])
        # get nearest lanelet to target
        goal_lanelet: Lanelet = self._find_nearest_lanlet(target)

        if start_lanelet is None:
            self.planning_problem = None
            return ProblemStatus.BadStart
        elif goal_lanelet is None:
            self.planning_problem = None
            return ProblemStatus.BadTarget

        # if start and target point are on the same lanelet
        if goal_lanelet.lanelet_id == start_lanelet.lanelet_id:
            # check whether start and target point are on that lanelet
            self.planning_problem = None
            start_index = PathProviderCommonRoads.find_nearest_path_index(route=start_lanelet.center_vertices,
                                                                          compare_point=start, use_posestamped=False)
            end_index = PathProviderCommonRoads.find_nearest_path_index(route=start_lanelet.center_vertices,
                                                                        compare_point=target, use_posestamped=False)
            # if the target point is ahead of the start point
            if start_index > end_index:
                # split lanelet in between those two points to fix that issue for a further iteration
                # which is triggered after a BadLanelet status is received by the compute_route algorithm
                split_point = Point(start_lanelet.center_vertices[start_index][0],
                                    start_lanelet.center_vertices[start_index][0], 0)

                self.manager.update_network(matching_lanelet_id=goal_lanelet.lanelet_id, modify_point=split_point,
                                            start_point=start, static_obstacle=None)

                self.planning_problem = None
                return ProblemStatus.BadLanelet

            # else the start point is ahead of the target point -> route consists of the center points of that lanelet
            # which thus are in the right order

        # create start and goal state for planning problem
        start_position = []
        if len(matching_lanelet[0]) == 0:
            start_position = [start_lanelet.center_vertices[len(start_lanelet.center_vertices) // 2][0],
                              start_lanelet.center_vertices[len(start_lanelet.center_vertices) // 2][1]]
        else:
            start_position = [start.x, start.y]

        start_state: State = State(position=np.array([start_position[0], start_position[1]]), velocity=0,
                                   time_step=0, slip_angle=0, yaw_rate=0, orientation=self.start_orientation[2])

        circle = Circle(5, center=np.array([goal_lanelet.center_vertices[len(goal_lanelet.center_vertices) // 2][0],
                                            goal_lanelet.center_vertices[len(goal_lanelet.center_vertices) // 2][1]]))
        goal_state: State = State(position=circle, time_step=Interval(0, 10000.0))

        goal_region: GoalRegion = GoalRegion([goal_state], None)

        # return planning problem with start_state and goal_region
        self.planning_problem = PlanningProblem(0, start_state, goal_region)
        return ProblemStatus.Success

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
        plt.savefig("route_" + str(num) + ".png")
        plt.close()

    @staticmethod
    def find_nearest_path_index(route, compare_point: Point, use_posestamped: bool = True,
                                reversed: bool = False, prematured_stop: bool = False):
        """
        Get index of the a point x in path which is next to a given point y
        :param route: List of route points, which are eather of type [x][y] or PoseStamped
        :param compare_point: Point to compare the route_point -> Type Point
        :param use_posestamped:  route_point is a PoseStamped -> Type Bool
        :return: index
        """
        min_distance = float("inf")
        min_index = -1
        higher_cnt = 0

        for i in range(0, len(route)):
            index = i
            if reversed:
                index = len(route) - 1 - i
            temp_distance = PathProviderCommonRoads._euclidean_2d_distance_from_to_position(route_point=route[index],
                                                                                            compare_point=compare_point,
                                                                                            use_posestamped=use_posestamped)
            if temp_distance < min_distance:
                min_distance = temp_distance
                min_index = index
            elif prematured_stop:
                higher_cnt += 1
                if higher_cnt == 5:
                    break

        return min_index

    @staticmethod
    def _euclidean_2d_distance_from_to_position(route_point, compare_point: Point, use_posestamped: bool = True):
        """
        This helper function calculates the euclidean distance between 2 Points
        :param route_point: Has to be the point entry in path. -> PoseStamped
        :param compare_point: Point to compare the route_point -> Type Point
        :param use_posestamped:  route_point is a PoseStamped -> Type Bool
        :return: distance
        """
        if use_posestamped:
            return ((route_point.pose.position.x - compare_point.x) ** 2 +
                    (route_point.pose.position.y - compare_point.y) ** 2) ** 0.5
        else:
            return ((route_point[0] - compare_point.x) ** 2 +
                    (route_point[1] - compare_point.y) ** 2) ** 0.5

    def _get_shortest_route(self, routes_list: list):
        """
        Heuristik to get the shortest route among all routes in route_list
        :param routes_list: list, which contains every generated route
        :return: shortest route determined by our heuristic
        """
        min_value = float("inf")
        best_path = None
        for route in routes_list:
            traffic_lights_cnt = 0
            time_spent = 0
            extended_path = self._generate_extended_route_path(route, skip_id=True)
            for lane in extended_path.route:
                time_spent += lane.time_spent
                traffic_lights_cnt += int(lane.hasLight)

            # heuristic adds defined amount of seconds for each traffic light
            temp_value = time_spent + traffic_lights_cnt * 30

            if temp_value < min_value:
                min_value = temp_value
                best_path = extended_path

        return best_path

    def _generate_extended_route_path(self, route: Route, skip_id=False):
        lane_change_instructions = route._compute_lane_change_instructions()
        extended_route = XRoute(id=self.route_id, route=[])
        if not skip_id:
            self.route_id += 1
        do_lane_change = False
        for i, lane_id in enumerate(route.list_ids_lanelets):
            if lane_change_instructions[i] == 0:
                if not do_lane_change:
                    extended_route.route.append(deepcopy(self.manager.message_by_lanelet[lane_id]))
                else:
                    message = deepcopy(self.manager.message_by_lanelet[lane_id])
                    message.route_portion = message.route_portion[len(message.route_portion) // 2:]
                    extended_route.route.append(deepcopy(message))
                    do_lane_change = False
            else:
                if not do_lane_change:
                    message = deepcopy(self.manager.message_by_lanelet[lane_id])
                    message.route_portion = message.route_portion[:len(message.route_portion) // 2]
                    extended_route.route.append(deepcopy(message))
                else:
                    rospy.logerr("This should not have happened, CHECK!")  # TODO: Check if it occurs

                do_lane_change = True
        return extended_route

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
        status = self._generate_planning_problem(start_point, target_point)

        # now check planning_problem
        if status == ProblemStatus.BadLanelet:
            status = self._generate_planning_problem(start_point, target_point)

        if status is ProblemStatus.BadStart:
            # planning_problem is None if e.g. start or goal position has not been found
            # Creates a empty path message, which is by consent filled with, and only with the starting_point
            rospy.logerr("PathProvider: Path computation aborted, insufficient start_point")
            self._create_path_message([self._get_pose_stamped(start_point, start_point)])
            return
        elif status is ProblemStatus.BadTarget:
            rospy.logerr("PathProvider: Path computation aborted, insufficient target_point")
            self._create_path_message([self._get_pose_stamped(start_point, start_point)])
            return
        elif status is ProblemStatus.BadLanelet:
            rospy.logerr("PathProvider: Path computation aborted, Lanelet Network Error -> Contact Support")
            self._create_path_message([self._get_pose_stamped(start_point, start_point)])
            return
        elif status is ProblemStatus.BadMap:
            rospy.logerr("PathProvider: Path computation aborted, Map not (yet) loaded")
            self._create_path_message([self._get_pose_stamped(start_point, start_point)])
            return

        # COMPUTE SHORTEST ROUTE
        # instantiate a route planner
        route_planner = RoutePlanner(self.manager.map, self.planning_problem,
                                     backend=RoutePlanner.Backend.PRIORITY_QUEUE)

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
            x_route = self._get_shortest_route(all_routes)

            # Path was found!
            # Prune path to get exact start and end points
            real_start_index = PathProviderCommonRoads.find_nearest_path_index(x_route.route[0].route_portion,
                                                                               start_point, prematured_stop=True,
                                                                               use_posestamped=False)
            x_route.route[0].route_portion = x_route.route[0].route_portion[real_start_index:]
            real_end_index = PathProviderCommonRoads.find_nearest_path_index(x_route.route[-1].route_portion,
                                                                             target_point, prematured_stop=True,
                                                                             use_posestamped=False)
            x_route.route[-1].route_portion = x_route.route[-1].route_portion[:real_end_index]

            for lane in x_route.route:
                prev_local_point = None
                for point in lane:
                    local_point = Point(point.x, point.y, 0)
                    if prev_local_point is None:
                        prev_local_point = local_point

                    path_poses.append(self._get_pose_stamped(local_point, prev_local_point))
                    prev_local_point = local_point

            rospy.loginfo("PathProvider: Computation of feasible path done")

        else:
            # Creates a empty path message, which is by consent filled with, and only with the starting_point
            path_poses = [self._get_pose_stamped(start_point, start_point)]
            rospy.logerr("PathProvider: No possible path was found")

        # save current path_poses
        self.path_poses = path_poses

        # create self.path messages
        self._create_path_message(path_poses, debug)

    def _reset_map(self):
        # first reset map
        if self.manager is not None:
            self.manager.map = deepcopy(self.manager.original_map)
