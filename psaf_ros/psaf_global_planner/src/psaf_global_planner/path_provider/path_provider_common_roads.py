#!/usr/bin/env python
import math
import pathlib

import rosbag
from lanelet2.core import GPSPoint
from tf.transformations import quaternion_from_euler, euler_from_quaternion

from psaf_abstraction_layer.VehicleStatus import VehicleStatusProvider
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from commonroad.scenario.trajectory import State
from commonroad.planning.goal import GoalRegion
import matplotlib.pyplot as plt

from commonroad.geometry.shape import Circle
from commonroad.scenario.lanelet import Lanelet
from commonroad.planning.planning_problem import PlanningProblem
from commonroad.common.util import Interval

from SMP.route_planner.route_planner.route_planner import RoutePlanner
from SMP.route_planner.route_planner.route import Route
import numpy as np
from SMP.route_planner.route_planner.utils_visualization import draw_route, get_plot_limits_from_reference_path
from psaf_global_planner.path_provider.common_road_manager import CommonRoadManager
from geometry_msgs.msg import PoseStamped, Point, Quaternion, Pose
import sys
from copy import deepcopy
import rospy
from psaf_global_planner.map_provider.map_supervisor_common_roads import MapSupervisorCommonRoads
from psaf_abstraction_layer.sensors.GPS import GPS_Sensor
from enum import Enum
from psaf_messages.msg import XRoute
from lanelet2.projection import UtmProjector
from lanelet2.io import Origin
from std_msgs.msg import String
import actionlib
from psaf_messages.msg import PlanningInstruction


class ProblemStatus(Enum):
    Success = 1
    BadTarget = 2
    BadStart = 3
    BadLanelet = 4
    BadMap = 5
    BadUTurn = 6


class PathProviderCommonRoads:

    def __init__(self, init_rospy: bool = False, polling_rate: int = 1, timeout_iter: int = 60,
                 role_name: str = "ego_vehicle",
                 initial_search_radius: float = 1.0, step_size: float = 1.0,
                 max_radius: float = 100, enable_debug: bool = False, cost_traffic_light: int = 30,
                 cost_stop_sign: int = 5, respect_traffic_rules: bool = False, turning_circle: float = 10.0,
                 export_path: bool = False, cost_lane_change: int = 5, line_crossing_penalty: int = 30,
                 cost_line_crossing: int = 15):
        if init_rospy:
            # initialize node
            rospy.init_node('pathProvider', anonymous=True)
        self.respect_traffic_rules = respect_traffic_rules
        self.enable_debug = enable_debug
        self.role_name = role_name
        self.GPS_Sensor = GPS_Sensor(role_name=self.role_name)
        self.origin = Origin(0, 0)
        self.projector = UtmProjector(self.origin)
        self.map_provider = MapSupervisorCommonRoads(debug=enable_debug)
        self.vehicle_status = VehicleStatusProvider(role_name=self.role_name)
        self.status_pub = rospy.Publisher('/psaf/status', String, queue_size=10)
        self.status_pub.publish("PathProvider not ready")

        # search parameters for finding a nearest lanelet
        self.radius = initial_search_radius
        self.step_size = step_size
        self.max_radius = max_radius

        # define heuristic costs, considered in the path selection process
        self.cost_traffic_light = cost_traffic_light
        self.cost_stop_sign = cost_stop_sign
        self.cost_lane_change = cost_lane_change
        # total cost for line crossing is the heuristic cost and the penalty if traffic rules should be obeyed
        if self.respect_traffic_rules:
            self.cost_line_crossing = line_crossing_penalty + cost_line_crossing
        else:
            self.cost_line_crossing = cost_line_crossing

        self.path_message = XRoute()
        self.planning_problem = None
        self.manager = None
        self.manager = CommonRoadManager(self._load_scenario(polling_rate, timeout_iter),
                                         intersections=self.map_provider.intersection,
                                         map_name=self.map_provider.map_name)
        self.route_id = 1  # 1 is the first valid id, 0 is reserved as invalid
        rospy.Subscriber("/psaf/goal/set_instruction", PlanningInstruction, self._callback_goal)
        self.xroute_pub = rospy.Publisher('/psaf/xroute', XRoute, queue_size=10)
        self.status_pub.publish("PathProvider ready")
        self.u_turn_distances = []  # first entry is left, second entry is forward distance
        self.turning_circle = turning_circle
        self.export_path: bool = export_path

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

    def _find_nearest_lanelet(self, goal: Point) -> Lanelet:
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

    def _find_nearest_u_turn_lanelet(self, start: Point, start_lanelet: Lanelet):
        """
        Find the nearest opposite lanelet
        :param start: x,y,z coordinates of the current staring position
        :param start_lanelet: Lanelet matching the start point above
        :return: True if a opposite lanelet was found and the corresponding lanelet,
                 False if not and None
        """
        neighbour_found = False
        if self.u_turn_distances[0] < self.turning_circle or self.u_turn_distances[1] < self.turning_circle / 2:
            # if free space is smaller than our minimum turning_circle a u_turn is not possible
            return False, None
        # first check if start_lanelet has a left adjacent neighbour, which faces in the opposite direction
        temp_lanelet = start_lanelet
        while temp_lanelet.adj_left is not None:
            if not temp_lanelet.adj_left_same_direction:
                neighbour_found = True
                temp_lanelet = self.manager.map.lanelet_network.find_lanelet_by_id(temp_lanelet.adj_left)
                break
            temp_lanelet = self.manager.map.lanelet_network.find_lanelet_by_id(temp_lanelet.adj_left)
        if not neighbour_found:
            # no adjacent neighbour found, search for unknown neighbours

            center_point = start_lanelet.center_vertices[len(start_lanelet.center_vertices) // 2]
            temp_index = PathProviderCommonRoads.find_nearest_path_index(start_lanelet.center_vertices, start,
                                                                         prematured_stop=False, use_xcenterline=False)
            start_orientation = self.map_provider.get_lanelet_orientation_at_index(start_lanelet, temp_index)

            # get all lanelets nearby
            nearest = self.manager.map.lanelet_network.lanelets_in_proximity(np.array(center_point),
                                                                             self.u_turn_distances[0])
            for near_lanelet in nearest:
                temp_index = PathProviderCommonRoads.find_nearest_path_index(near_lanelet.center_vertices, start,
                                                                             prematured_stop=False,
                                                                             use_xcenterline=False)
                curr_orientation = self.map_provider.get_lanelet_orientation_at_index(near_lanelet, temp_index)

                # calculate angle_diff but consider that the two orientations should be opposite to each other
                start_angle_diff = abs(abs(start_orientation - curr_orientation) - 180)
                # do these two lanelets fulfill the orientation criteria
                if start_angle_diff < self.map_provider.max_neighbour_angle_diff:
                    temp_lanelet = near_lanelet
                    neighbour_found = True
                    break

        if neighbour_found:
            # a solution was found, check if it fulfills the distance constrains
            temp_index = PathProviderCommonRoads.find_nearest_path_index(temp_lanelet.center_vertices, start,
                                                                         prematured_stop=False, use_xcenterline=False)
            distance = np.linalg.norm(temp_lanelet.center_vertices[temp_index] - np.array([start.x, start.y]))
            if distance > self.u_turn_distances[0]:
                # u_turn not possible
                return False, None
            else:
                # everything is fine
                return True, temp_lanelet
        else:
            # no solution was found
            return False, None

    def _get_matching_lanelet(self, pos: Pose):
        # check if the car is already on a lanelet, if not get nearest lanelet to start
        matching_lanelet = self.manager.map.lanelet_network.find_lanelet_by_position(
            [np.array([pos.position.x, pos.position.y])])
        if len(matching_lanelet[0]) == 0:
            # no matching lanelet found -> search nearby
            res_lanelet = [self._find_nearest_lanelet(pos.position)]
        elif len(matching_lanelet[0]) == 1:
            # one matching lanelet found -> return that lanelet
            res_lanelet = [self.manager.map.lanelet_network.find_lanelet_by_id(matching_lanelet[0][0])]
        else:
            # more than one matching lanelet found -> search for lanelet with the correct orientation
            matching_candidates = []
            # get start orientation as yaw angle
            matching_yaw = math.degrees(euler_from_quaternion((pos.orientation.x, pos.orientation.y,
                                                            pos.orientation.z, pos.orientation.w))[2])
            # mirror and get positive angle to match yaw direction of map provider
            matching_yaw *= -1
            if matching_yaw < 0:
                matching_yaw += 360
            for lanelet_id in matching_lanelet[0]:
                temp_lanelet = self.manager.map.lanelet_network.find_lanelet_by_id(lanelet_id)
                temp_index = PathProviderCommonRoads.find_nearest_path_index(temp_lanelet.center_vertices,
                                                                             pos.position, prematured_stop=False,
                                                                             use_xcenterline=False)
                temp_orientation = self.map_provider.get_lanelet_orientation_at_index(temp_lanelet, temp_index)
                matching_candidates.append([temp_lanelet, abs(temp_orientation - matching_yaw)])

            # calculate orientation differences for every lanelet candidate
            matching_candidates.sort(key=lambda x: x[1])
            # start_lanelet is a list of matching lanelet with the least orientation difference
            res_lanelet = [matching_candidates[0][0]]
            for index, candidate in enumerate(matching_candidates):
                if abs(candidate[1] < 10) and index != 0:
                    res_lanelet.append(candidate[0])

        return res_lanelet

    def _generate_planning_problem(self, start: Pose, target: Pose, u_turn: bool = False) -> ProblemStatus:
        """
        Generate the planning problem by setting the starting point and generating a target region
        :param start:  Pose: x,y,z coordinates of the current staring position and orientation
        :param target: Pose x,y,z coordinates of the current goal position and orientation
        :param u_turn: if True plan route with a initial u_turn
        :return: status of the generated problem
        """
        if self.manager.map is None:
            # No map -> generating of planning problem not possible
            self.status_pub.publish("No map -> Planning aborted")
            self.planning_problem = None
            return ProblemStatus.BadMap

        # get start lanelets
        start_lanelet_list = self._get_matching_lanelet(start)

        # get nearest lanelets to target
        goal_lanelet_list = self._get_matching_lanelet(target)

        if start_lanelet_list[0] is None:
            self.planning_problem = None
            return ProblemStatus.BadStart
        if goal_lanelet_list[0] is None:
            self.planning_problem = None
            return ProblemStatus.BadTarget

        start_lanelet = start_lanelet_list[0]
        goal_lanelet = goal_lanelet_list[0]

        rospy.loginfo("PathProvider: Matching Lanelet (Start) ID " + str(start_lanelet.lanelet_id))
        rospy.loginfo("PathProvider: Matching Lanelet (End) ID " + str(goal_lanelet.lanelet_id))

        if u_turn:
            u_turn_success, start_lanelet = self._find_nearest_u_turn_lanelet(start.position, start_lanelet)
            if not u_turn_success:
                return ProblemStatus.BadUTurn

        # if start and target point are on the same lanelet
        if goal_lanelet.lanelet_id == start_lanelet.lanelet_id:
            # check whether start and target point are on that lanelet
            self.planning_problem = None
            start_index = PathProviderCommonRoads.find_nearest_path_index(route=start_lanelet.center_vertices,
                                                                          compare_point=start.position,
                                                                          use_xcenterline=False)
            end_index = PathProviderCommonRoads.find_nearest_path_index(route=start_lanelet.center_vertices,
                                                                        compare_point=target.position,
                                                                        use_xcenterline=False)
            # if the target point is ahead of the start point
            if start_index > end_index:
                # split lanelet in between those two points to fix that issue for a further iteration
                # which is triggered after a BadLanelet status is received by the compute_route algorithm
                split_index = end_index + (start_index - end_index) // 2
                split_point = Point(start_lanelet.center_vertices[split_index][0],
                                    start_lanelet.center_vertices[split_index][1], 0)

                self.manager.update_network(matching_lanelet_id=start_lanelet.lanelet_id, modify_point=split_point,
                                            start_point=start.position, static_obstacle=None)

                self.planning_problem = None
                return ProblemStatus.BadLanelet

            # else the start point is ahead of the target point -> route consists of the center points of that lanelet
            # which thus are in the right order

        # create start and goal state for planning problem
        index = PathProviderCommonRoads.find_nearest_path_index(start_lanelet.center_vertices, start.position,
                                                                prematured_stop=False, use_xcenterline=False)

        # yaw in third entry ([2]) of euler notation
        start_yaw = euler_from_quaternion((start.orientation.x, start.orientation.y,
                                           start.orientation.z, start.orientation.w))[2]
        # if u turn mirror direction
        if u_turn:
            start_yaw += math.pi
            start_yaw = math.fmod(start_yaw, 2 * math.pi)
        start_state: State = State(position=start_lanelet.center_vertices[index], velocity=0,
                                   time_step=0, slip_angle=0, yaw_rate=0,
                                   orientation=start_yaw)

        circle = Circle(1, center=np.array([goal_lanelet.center_vertices[len(goal_lanelet.center_vertices) // 2][0],
                                            goal_lanelet.center_vertices[len(goal_lanelet.center_vertices) // 2][1]]))
        goal_state: State = State(position=circle, time_step=Interval(0, 10000.0))

        # collect the ids of all possible goal lanelets
        lanelet_ids = []
        for lane in goal_lanelet_list:
            lanelet_ids.append(lane.lanelet_id)

        goal_region: GoalRegion = GoalRegion(state_list=[goal_state],
                                             lanelets_of_goal_position={0: lanelet_ids})

        # return planning problem with start_state and goal_region
        self.planning_problem = PlanningProblem(0, start_state, goal_region)
        return ProblemStatus.Success

    def get_path_from_a_to_b(self, from_a: Pose = None, to_b: Pose = None, u_turn: bool = False):
        """
        Returns the shortest path from start to goal
        :param  from_a: Start point [Pose] optional
        :param  to_b:   End point [Pose] optional
        :param u_turn: if True: check if a u_turn at the start is beneficial
        :return: Path or only start if no path was found at all, or no map information is received
        """
        start = self.start
        goal = self.goal

        if from_a is not None and to_b is not None:
            start = from_a
            goal = to_b

        if u_turn:
            x_route_u, best_value_u = self._compute_route(start, goal, u_turn=True)
            # add line crossing cost to the value of the u_turn path
            best_value_u += self.cost_line_crossing
            x_route_no_u, best_value_no_u = self._compute_route(start, goal, u_turn=False)
            # compare distance based on obey_traffic_rules param and save best
            if best_value_u < best_value_no_u:
                x_route = x_route_u
            else:
                x_route = x_route_no_u
        else:
            x_route, _ = self._compute_route(start, goal, u_turn=False)

        # if the path is valid
        if x_route.id > 0:
            # publish message and trigger the global planner plugin
            self.xroute_pub.publish(x_route)
            # save message
            self.path_message = x_route

            # trigger move_base with dummy point
            dummy_point = Point(0, 0, 0)
            self._trigger_move_base(self._get_pose_stamped(dummy_point, dummy_point))
        else:
            rospy.logerr("PathProvider: Invalid path, no path published")

    def _visualization(self, route, num):
        """
        Create a .png file, visualizing the given route (only used if enable_debug is True)
        :param route: the route to be visualized
        :param num: number of the route (only for identification purposes)
        """
        plot_limits = get_plot_limits_from_reference_path(route)
        # option 2: plot limits from lanelets in the route
        # plot_limits = get_plot_limits_from_routes(route)
        rospy.loginfo("Visualize")
        # determine the figure size for better visualization
        size_x = 10
        ratio_x_y = (plot_limits[1] - plot_limits[0]) / (plot_limits[3] - plot_limits[2])
        fig = plt.figure(figsize=(size_x, size_x / ratio_x_y))
        fig.gca().axis('equal')
        draw_route(route, draw_route_lanelets=True, draw_reference_path=True, plot_limits=plot_limits)
        plt.savefig("route_" + str(num) + ".png")
        plt.close()

    @staticmethod
    def find_nearest_path_index(route, compare_point: Point, use_xcenterline: bool = True,
                                reversed: bool = False, prematured_stop: bool = False):
        """
        Get index of the a point x in path which is next to a given point y
        :param route: List of route points, which are eather of type [x][y] or xcenterline
        :param compare_point: Point to compare the route_point -> Type Point
        :param use_xcenterline:  route_point is a xcenterline entry
        :param reversed:  if true reverse search order (route[-1]-> route[-2] ...)
        :param prematured_stop:  if true stop search after a match was found, may produce a global minimum
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
                                                                                            use_xcenterline=use_xcenterline)
            if temp_distance < min_distance:
                min_distance = temp_distance
                min_index = index
            elif prematured_stop:
                higher_cnt += 1
                if higher_cnt == 5:
                    break

        return min_index

    @staticmethod
    def _euclidean_2d_distance_from_to_position(route_point, compare_point: Point, use_xcenterline: bool = True):
        """
        This helper function calculates the euclidean distance between 2 Points
        :param route_point: Has to be the point entry in path. -> xcenterline
        :param compare_point: Point to compare the route_point -> Type Point
        :param use_xcenterline:  route_point is a xcenterline -> Type Bool
        :return: distance
        """
        if use_xcenterline:
            return ((route_point.x - compare_point.x) ** 2 +
                    (route_point.y - compare_point.y) ** 2) ** 0.5
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
            extended_path, distance, time_spent = self._generate_extended_route_path(route)
            if self.enable_debug:
                for xlane in extended_path.route:
                    for portion in xlane.route_portion:
                        route.reference_path.append([portion.x, portion.y])
                route.reference_path = np.array(route.reference_path)
            if self.respect_traffic_rules:
                if time_spent < min_value:
                    min_value = time_spent
                    best_path = extended_path
            else:
                if distance < min_value:
                    min_value = distance
                    best_path = extended_path
        self.route_id += 1
        best_path.id = self.route_id
        return best_path, min_value

    def _generate_extended_route_path(self, route: Route):
        """
        Generate the extended_route_path and provide a heuristic evaluation of the time,
        passed while traveling on that route. Additional the covered distance is calculated.
        :param route: Route
        :return: extended route and its corresponding time duration heuristic
        """
        lane_change_instructions = route._compute_lane_change_instructions()
        extended_route = XRoute()
        do_lane_change = False
        time = 0
        dist = 0
        for i, lane_id in enumerate(route.list_ids_lanelets):
            message = deepcopy(self.manager.message_by_lanelet[lane_id])
            if lane_change_instructions[i] == 0:
                # no lane change -> isLaneChange is False by default

                if not do_lane_change:
                    extended_route.route.append(deepcopy(message))
                    time += message.route_portion[-1].duration
                    dist += message.route_portion[-1].distance

                    # heuristic adds defined amount of seconds for each traffic light
                    time += int(message.hasLight) * self.cost_traffic_light
                    time += int(message.hasStop) * self.cost_stop_sign
                else:
                    tmp_message = deepcopy(message)
                    tmp_message.route_portion = message.route_portion[len(message.route_portion) // 2:]
                    # make sure to correct entries in route_portion
                    for waypoint in tmp_message.route_portion:
                        waypoint.duration = waypoint.duration - \
                                            message.route_portion[len(message.route_portion) // 2].duration
                        waypoint.distance = waypoint.distance - \
                                            message.route_portion[len(message.route_portion) // 2].distance

                    extended_route.route.append(deepcopy(tmp_message))

                    time += message.route_portion[-1].duration - message.route_portion[
                        len(message.route_portion) // 2].duration
                    dist += message.route_portion[-1].distance - message.route_portion[
                        len(message.route_portion) // 2].distance

                    # heuristic adds defined amount of seconds for each traffic light and stop signs on the lanelet
                    time += int(message.hasLight) * self.cost_traffic_light
                    time += int(message.hasStop) * self.cost_stop_sign

                    do_lane_change = False
            else:
                if not do_lane_change:
                    # lane change -> set isLaneChange True
                    message.isLaneChange = True
                    # lane change occurs in front of a intersection -> no traffic signs or lights
                    message.hasStop = False
                    message.hasLight = False
                    message.route_portion = message.route_portion[:len(message.route_portion) // 2]
                    extended_route.route.append(deepcopy(message))
                    time += message.route_portion[-1].duration
                    # add lane change penalty
                    time += self.cost_lane_change
                    dist += message.route_portion[-1].distance
                else:
                    # lane changing over at least two lanes at once -> do not count the skipped middle lane
                    pass

                do_lane_change = True

        return extended_route, dist, time

    def _compute_route(self, from_a: Pose, to_b: Pose, u_turn: bool = False):
        """
        Compute shortest path
        :param from_a: Start Pose
        :param to_b: End Pose
        :param u_turn: if True plan route with a initial u_turn
        """""
        x_route = XRoute(isUTurn=u_turn)
        best_value = float("inf")

        if u_turn:
            rospy.loginfo("PathProvider: Computing feasible path from a to b, with initial U_Turn")
        else:
            rospy.loginfo("PathProvider: Computing feasible path from a to b")

        # first generate planning_problem
        status = self._generate_planning_problem(from_a, to_b, u_turn=u_turn)

        # now check planning_problem
        if status == ProblemStatus.BadLanelet:
            status = self._generate_planning_problem(from_a, to_b, u_turn=u_turn)

        if status is ProblemStatus.BadStart:
            # planning_problem is None if e.g. start or goal position has not been found
            rospy.logerr("PathProvider: Path computation aborted, insufficient start_point")
            self.status_pub.publish("Path computation aborted, insufficient start_point")
            return x_route, best_value
        elif status is ProblemStatus.BadTarget:
            rospy.logerr("PathProvider: Path computation aborted, insufficient target_point")
            self.status_pub.publish("Path computation aborted, insufficient target_point")
            return x_route, best_value
        elif status is ProblemStatus.BadLanelet:
            rospy.logerr("PathProvider: Path computation aborted, Lanelet Network Error -> Contact Support")
            self.status_pub.publish("Path computation aborted, Lanelet Network Error")
            return x_route, best_value
        elif status is ProblemStatus.BadMap:
            rospy.logerr("PathProvider: Path computation aborted, Map not (yet) loaded")
            self.status_pub.publish("Path computation aborted, Map not (yet) loaded")
            return x_route, best_value
        elif status is ProblemStatus.BadUTurn:
            rospy.logerr("PathProvider: Path computation aborted, UTurn not possible")
            self.status_pub.publish("Path computation aborted, UTurn not possible")
            return x_route, best_value

        # COMPUTE SHORTEST ROUTE
        # instantiate a route planner
        route_planner = RoutePlanner(self.manager.map, self.planning_problem,
                                     backend=RoutePlanner.Backend.PRIORITY_QUEUE)

        # plan routes, and save the found routes in a route candidate holder
        candidate_holder = route_planner.plan_routes()
        all_routes, num_routes = candidate_holder.retrieve_all_routes()

        if num_routes >= 1:
            # Path was found!
            x_route, best_value = self._get_shortest_route(all_routes)
            x_route.isUTurn = u_turn

            # Prune path to get exact start and end points
            x_route = self._prune_path(x_route, from_a.position, to_b.position)

            if self.enable_debug:
                # if debug: show results in .png
                for i in range(0, num_routes):
                    self._visualization(all_routes[i], i)

            if len(x_route.route) == 1 and len(x_route.route[0].route_portion) == 0:
                # already on target point, invalidate path
                rospy.logerr("PathProvider: Already on target point, no waypoints generated")
                x_route.id = 0

        else:
            rospy.logerr("PathProvider: No possible path was found")

        if self.export_path:
            self._serialize_message(x_route=x_route)

        return x_route, best_value

    def _serialize_message(self, x_route: XRoute):
        """
        Serialize XRoute message and write to bag file, for use in the scenario runner
        :param x_route: XRoute to be serialized
        """
        # suffix set to be .debug
        suffix = ".debugpath"
        # debug directory, common parent folder is four directory levels above current folder
        dir_str = str(pathlib.Path(__file__).parent.absolute().parents[3]) + "/psaf_scenario/scenarios/"
        bag_file = rosbag.Bag(str(dir_str + "path" + suffix), "w")
        bag_file.write('XRoute', x_route)
        bag_file.close()

        rospy.loginfo("PathProvider: Created bag file with path. ../psaf_scenario/scenarios/")

    def _prune_path(self, path: XRoute, start: Point, target: Point):
        """
        Prunes the path by the given start and target point
        :param path: Path to be pruned
        :param start:  x,y,z coordinates of the current starting position
        :param target: x,y,z coordinates of the current goal position
        :return: pruned path
        """
        real_start_index = PathProviderCommonRoads.find_nearest_path_index(path.route[0].route_portion,
                                                                           start,
                                                                           prematured_stop=False,
                                                                           use_xcenterline=True)
        path.route[0].route_portion = path.route[0].route_portion[real_start_index:]

        # If there is a laneChange on the first lanelet, a point on the second lanelet might be nearer, because
        # we are already past the laneChange..
        # That is only the case exactly when the length of the route_portion of the first lanelet is one, because
        # its last entry is the closest. --> objective: put laneChange after our current position
        if len(path.route[0].route_portion) == 1 and path.route[0].isLaneChange:
            # get the portion of the whole lanelet
            real_route_portion = self.manager.message_by_lanelet[path.route[0].id].route_portion
            # get its corresponding true start_index
            real_start_index = PathProviderCommonRoads.find_nearest_path_index(real_route_portion,
                                                                               start,
                                                                               prematured_stop=False,
                                                                               use_xcenterline=True)
            # add the two previous points
            path.route[0].route_portion[0] = real_route_portion[real_start_index - 1]
            path.route[0].route_portion.append(real_route_portion[real_start_index])

            # get the true point on the following lanelet (after lane changing)
            real_start_index = PathProviderCommonRoads.find_nearest_path_index(path.route[1].route_portion,
                                                                               start,
                                                                               prematured_stop=False,
                                                                               use_xcenterline=True)
            path.route[1].route_portion = path.route[1].route_portion[real_start_index:]

            # adjust duration and distance of the following lanelet (see below)
            for waypoint in reversed(path.route[1].route_portion):
                waypoint.duration = waypoint.duration - path.route[1].route_portion[0].duration
                waypoint.distance = waypoint.distance - path.route[1].route_portion[0].distance

        # adjust duration and distance of start lanelet to match the criteria of a cumulative sum, starting by zero
        for waypoint in reversed(path.route[0].route_portion):
            waypoint.duration = waypoint.duration - path.route[0].route_portion[0].duration
            waypoint.distance = waypoint.distance - path.route[0].route_portion[0].distance

        real_end_index = PathProviderCommonRoads.find_nearest_path_index(path.route[-1].route_portion,
                                                                         target, prematured_stop=False,
                                                                         use_xcenterline=True)
        path.route[-1].route_portion = path.route[-1].route_portion[:real_end_index+1]

        return path

    def _callback_goal(self, data):
        """
        Callback function of psaf goal set subscriber
        :param data: data received
        """
        self._reset_map()
        self.goal = Pose(position=data.goalPoint.position, orientation=data.goalPoint.orientation)
        start = self.GPS_Sensor.get_position()
        start = self.projector.forward(GPSPoint(start.latitude, start.longitude, start.altitude))
        start_orientation = self.vehicle_status.get_status().orientation
        self.start = Pose(position=Point(x=start.x, y=start.y, z=start.z), orientation=start_orientation)
        self.u_turn_distances = [data.obstacleDistanceLeft, data.obstacleDistanceForward]

        rospy.loginfo("PathProvider: Received start and goal position")
        self.get_path_from_a_to_b(u_turn=data.planUTurn)
        self.status_pub.publish("PathProvider done")

    def _trigger_move_base(self, trigger_pose: PoseStamped):
        """
        This function triggers the move_base by publishing a PoseStamped
        :param trigger_pose: Pose
        """
        client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        client.wait_for_server()

        goal = MoveBaseGoal()
        goal.target_pose = trigger_pose

        client.send_goal(goal)
        rospy.loginfo("PathProvider: global planner plugin triggered")

    def _get_pose_stamped(self, pos: Point, prev_pos: Point):
        """
        This function calculates the euler angle alpha according to the direction vector of two given points.
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

        # describes the relative position of the pos to the prev pos
        rel_x = 1 if (pos.x - prev_pos.x) >= 0 else -1
        rel_y = 1 if (pos.y - prev_pos.y) >= 0 else -1

        euler_angle_yaw = math.atan2(rel_y * abs(pos.y - prev_pos.y), rel_x * abs(pos.x - prev_pos.x))

        # only 2D space is relevant, therefore angles beta and gamma can be set to zero
        q = quaternion_from_euler(0.0, 0.0, euler_angle_yaw)
        p.pose.orientation = Quaternion(*q)
        p.header.frame_id = "map"

        return p

    def _reset_map(self):
        """
        Reset the current knowledge to the unchanged originals
        """
        # first reset map
        if self.manager is not None:
            # create clean slate
            self.manager.map = deepcopy(self.manager.original_map)
            self.manager.neighbourhood = deepcopy(self.manager.original_neighbourhood)
            self.manager.message_by_lanelet = deepcopy(self.manager.original_message_by_lanelet)
