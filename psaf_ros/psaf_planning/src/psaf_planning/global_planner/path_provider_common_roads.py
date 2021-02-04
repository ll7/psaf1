#!/usr/bin/env python
import math

from lanelet2.core import GPSPoint
from tf.transformations import quaternion_from_euler

from psaf_abstraction_layer.VehicleStatus import VehicleStatusProvider
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
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
from psaf_planning.global_planner.common_road_manager import CommonRoadManager
from geometry_msgs.msg import PoseStamped, Point, Quaternion
import sys
from copy import deepcopy
import rospy
from sensor_msgs.msg import NavSatFix
from psaf_planning.map_provider.common_roads_map_provider_plus import CommonRoadMapProvider
from psaf_abstraction_layer.sensors.GPS import GPS_Position, GPS_Sensor
from enum import Enum
from psaf_messages.msg import XRoute
from lanelet2.projection import UtmProjector
from lanelet2.io import Origin
from std_msgs.msg import String
import actionlib


class ProblemStatus(Enum):
    Success = 1
    BadTarget = 2
    BadStart = 3
    BadLanelet = 4
    BadMap = 5


class PathProviderCommonRoads:

    def __init__(self, init_rospy: bool = False, polling_rate: int = 1, timeout_iter: int = 20,
                 role_name: str = "ego_vehicle",
                 initial_search_radius: float = 1.0, step_size: float = 1.0,
                 max_radius: float = 100, enable_debug: bool = False, cost_traffic_light: int = 30,
                 cost_stop_sign: int = 5):
        if init_rospy:
            # initialize node
            rospy.init_node('pathProvider', anonymous=True)
        self.enable_debug = enable_debug
        self.role_name = role_name
        self.GPS_Sensor = GPS_Sensor(role_name=self.role_name)
        self.origin = Origin(0, 0)
        self.projector = UtmProjector(self.origin)
        self.map_provider = CommonRoadMapProvider(debug=enable_debug)
        self.vehicle_status = VehicleStatusProvider(role_name=self.role_name)
        self.status_pub = rospy.Publisher('/psaf/status', String, queue_size=10)
        self.status_pub.publish("PathProvider not ready")
        self.radius = initial_search_radius
        self.step_size = step_size
        self.max_radius = max_radius
        self.cost_traffic_light = cost_traffic_light
        self.cost_stop_sign = cost_stop_sign
        self.path_message = XRoute()
        self.planning_problem = None
        self.manager = None
        self.manager = CommonRoadManager(self._load_scenario(polling_rate, timeout_iter))
        self.route_id = 1  # 1 is the first valid id, 0 is reserved as invalid
        rospy.Subscriber("/psaf/goal/set", NavSatFix, self._callback_goal)
        self.xroute_pub = rospy.Publisher('/psaf/xroute', XRoute, queue_size=10)
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
            start_lanelet = self._find_nearest_lanelet(start)
        else:
            start_lanelet = self.manager.map.lanelet_network.find_lanelet_by_id(matching_lanelet[0][0])
        # get nearest lanelet to target
        goal_lanelet: Lanelet = self._find_nearest_lanelet(target)

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
                                                                          compare_point=start, use_xcenterline=False)
            end_index = PathProviderCommonRoads.find_nearest_path_index(route=start_lanelet.center_vertices,
                                                                        compare_point=target, use_xcenterline=False)
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

        circle = Circle(1, center=np.array([goal_lanelet.center_vertices[len(goal_lanelet.center_vertices) // 2][0],
                                            goal_lanelet.center_vertices[len(goal_lanelet.center_vertices) // 2][1]]))
        goal_state: State = State(position=circle, time_step=Interval(0, 10000.0))

        goal_region: GoalRegion = GoalRegion([goal_state], None)

        # return planning problem with start_state and goal_region
        self.planning_problem = PlanningProblem(0, start_state, goal_region)
        return ProblemStatus.Success

    def get_path_from_a_to_b(self, from_a: GPS_Position = None, to_b: GPS_Position = None, u_turn: bool = False):
        """
        Returns the shortest path from start to goal
        :param  from_a: Start point [GPS Coord] optional
        :param  to_b:   End point [GPS Coord] optional
        :param u_turn: if True: check if a u_turn at the start is beneficial
        :return: Path or only start if no path was found at all, or no map information is received
        """
        start = self.start
        goal = self.goal

        if from_a is not None and to_b is not None:
            start = from_a
            goal = to_b

        if u_turn:
            x_route_u = self._compute_route(start, goal, u_turn=True)
            x_route_no_u = self._compute_route(start, goal, u_turn=False)
            # TODO: compare times or distance based on obey_traffic_rules param and save best
            x_route = x_route_u
        else:
            x_route = self._compute_route(start, goal, u_turn=False)

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
    def find_nearest_path_index(route, compare_point: Point, use_xcenterline: bool = True,
                                reversed: bool = False, prematured_stop: bool = False):
        """
        Get index of the a point x in path which is next to a given point y
        :param route: List of route points, which are eather of type [x][y] or PoseStamped
        :param compare_point: Point to compare the route_point -> Type Point
        :param use_xcenterline:  route_point is a PoseStamped -> Type Bool
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
        :param route_point: Has to be the point entry in path. -> PoseStamped
        :param compare_point: Point to compare the route_point -> Type Point
        :param use_xcenterline:  route_point is a PoseStamped -> Type Bool
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
            if time_spent < min_value:
                min_value = time_spent
                best_path = extended_path

        self.route_id += 1
        best_path.id = self.route_id
        return best_path

    def _generate_extended_route_path(self, route: Route):
        """
        Generate the extended_route_path and provide a heuristic evaluation of the time,
        passed while traveling on that route. Additional the covered distance is calculated.
        :param route: Route
        :return: extended route and its corresponding time duration heuristic
        """
        lane_change_instructions = route._compute_lane_change_instructions()
        extended_route = XRoute(id=self.route_id, route=[])
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
                    dist += message.route_portion[-1].dist - message.route_portion[
                        len(message.route_portion) // 2].dist

                    # heuristic adds defined amount of seconds for each traffic light and stop signs on the lanelet
                    time += int(message.hasLight) * self.cost_traffic_light
                    time += int(message.hasStop) * self.cost_stop_sign

                    do_lane_change = False
            else:
                # lane change -> set isLaneChange True
                message.isLaneChange = True
                if not do_lane_change:
                    tmp_message = deepcopy(message)
                    tmp_message.route_portion = message.route_portion[:len(message.route_portion) // 2]
                    extended_route.route.append(deepcopy(tmp_message))
                    time += message.route_portion[(len(message.route_portion) // 2)-1].duration
                    dist += message.route_portion[(len(message.route_portion) // 2)-1].distance
                else:
                    # lane changing over at least two lanes at once -> do not count the skipped middle lane
                    pass

                do_lane_change = True

        return extended_route, dist, time

    def _compute_route(self, from_a: GPS_Position, to_b: GPS_Position, u_turn: bool = False):
        """
        Compute shortest path
        :param from_a: Start point -- GPS Coord in float: latitude, longitude, altitude
        :param to_b: End point   -- GPS Coord in float: latitude, longitude, altitude
        :param u_turn: if True plan route with a initial u_turn
        """""
        if u_turn:
            rospy.loginfo("PathProvider: Computing feasible path from a to b, with initial U_Turn")
        else:
            rospy.loginfo("PathProvider: Computing feasible path from a to b")

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
            rospy.logerr("PathProvider: Path computation aborted, insufficient start_point")
            self.status_pub.publish("Path computation aborted, insufficient start_point")
            return
        elif status is ProblemStatus.BadTarget:
            rospy.logerr("PathProvider: Path computation aborted, insufficient target_point")
            self.status_pub.publish("Path computation aborted, insufficient target_point")
            return
        elif status is ProblemStatus.BadLanelet:
            rospy.logerr("PathProvider: Path computation aborted, Lanelet Network Error -> Contact Support")
            self.status_pub.publish("Path computation aborted, Lanelet Network Error")
            return
        elif status is ProblemStatus.BadMap:
            rospy.logerr("PathProvider: Path computation aborted, Map not (yet) loaded")
            self.status_pub.publish("Path computation aborted, Map not (yet) loaded")
            return

        # COMPUTE SHORTEST ROUTE
        # instantiate a route planner
        route_planner = RoutePlanner(self.manager.map, self.planning_problem,
                                     backend=RoutePlanner.Backend.PRIORITY_QUEUE)

        # plan routes, and save the found routes in a route candidate holder
        candidate_holder = route_planner.plan_routes()
        all_routes, num_routes = candidate_holder.retrieve_all_routes()

        x_route = XRoute()
        if num_routes >= 1:
            # Path was found!
            x_route = self._get_shortest_route(all_routes)

            # Prune path to get exact start and end points
            real_start_index = PathProviderCommonRoads.find_nearest_path_index(x_route.route[0].route_portion,
                                                                               start_point,
                                                                               prematured_stop=True,
                                                                               use_xcenterline=True)
            x_route.route[0].route_portion = x_route.route[0].route_portion[real_start_index:]

            # adjust duration entry of start lanelet to match the criteria of a cumulative sum, starting by zero
            for waypoint in reversed(x_route.route[0].route_portion):
                waypoint.duration = waypoint.duration - \
                                    x_route.route[0].route_portion[0].duration

            real_end_index = PathProviderCommonRoads.find_nearest_path_index(x_route.route[-1].route_portion,
                                                                             target_point, prematured_stop=False,
                                                                             use_xcenterline=True)
            x_route.route[-1].route_portion = x_route.route[-1].route_portion[:real_end_index]

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
            return

        if self.enable_debug:
            # TODO: serialize message for optimizer
            pass

        return x_route

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
        self.get_path_from_a_to_b()
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
        # first reset map
        if self.manager is not None:
            # create clean slate
            self.manager.map = deepcopy(self.manager.original_map)
            self.manager.neighbourhood = deepcopy(self.manager.original_neighbourhood)
            self.manager.message_by_lanelet = deepcopy(self.manager.original_message_by_lanelet)
