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
from geometry_msgs.msg import Point
import sys
from copy import deepcopy
import rospy

from enum import Enum


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
                 max_radius: float = 100, enable_debug: bool = False):
        super(PathProviderCommonRoads, self).__init__(init_rospy, polling_rate, timeout_iter, role_name,
                                                      enable_debug=enable_debug)
        self.radius = initial_search_radius
        self.step_size = step_size
        self.max_radius = max_radius

        # load map and store the original_map which is not going to be altered
        self.map = self._load_scenario(polling_rate, timeout_iter)
        self.original_map = deepcopy(self.map)

        self.path_poses = []
        self.planning_problem = None

        # create neighbourhood dicts for efficient access to that information
        self.neighbourhood = self._analyze_neighbourhood(self.original_map)
        self.original_neighbourhood = deepcopy(self.neighbourhood)

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
            nearest = self.map.lanelet_network.lanelets_in_proximity(np.array([goal.x, goal.y]), curr_radius)
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
        if self.map is None:
            # No map -> generating of planning problem not possible
            self.status_pub.publish("No map -> Planning aborted")
            self.planning_problem = None
            return ProblemStatus.BadMap

        # check if the car is already on a lanelet, if not get nearest lanelet to start
        start_lanelet: Lanelet
        matching_lanelet = self.map.lanelet_network.find_lanelet_by_position([np.array([start.x, start.y])])
        if len(matching_lanelet[0]) == 0:
            start_lanelet = self._find_nearest_lanlet(start)
        else:
            start_lanelet = self.map.lanelet_network.find_lanelet_by_id(matching_lanelet[0][0])
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
            start_index = self._find_nearest_path_index(route=start_lanelet.center_vertices,
                                                        compare_point=start, use_posestamped=False)
            end_index = self._find_nearest_path_index(route=start_lanelet.center_vertices,
                                                      compare_point=target, use_posestamped=False)
            # if the target point is ahead of the start point
            if start_index > end_index:
                # split lanelet in between those two points to fix that issue for a further iteration
                # which is triggered after a BadLanelet status is received by the compute_route algorithm
                split_point = Point(start_lanelet.center_vertices[start_index][0],
                                    start_lanelet.center_vertices[start_index][0], 0)

                self._update_network(matching_lanelet_id=goal_lanelet.lanelet_id, modify_point=split_point,
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

    def _find_nearest_path_index(self, route, compare_point: Point, use_posestamped: bool = True,
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
            temp_distance = self._euclidean_2d_distance_from_to_position(route_point=route[index],
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

    def _euclidean_2d_distance_from_to_position(self, route_point, compare_point: Point, use_posestamped: bool = True):
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
        route_planner = RoutePlanner(self.map, self.planning_problem, backend=RoutePlanner.Backend.PRIORITY_QUEUE)

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
            real_start_index = self._find_nearest_path_index(path_poses, start_point, prematured_stop=True)
            real_end_index = self._find_nearest_path_index(path_poses, target_point, True, True)
            path_poses_shortened = path_poses[real_start_index:real_end_index]

            rospy.loginfo("PathProvider: Points in path: {}".format(len(path_poses_shortened)))
            # check if you are already closer to end point than to start point (rare condition)
            if len(path_poses_shortened) == 0:
                path_poses_shortened.append(path_poses[real_end_index])
                rospy.logerr("PathProvider: Already close to target point, no path output was generated")
            else:
                rospy.loginfo("PathProvider: Computation of feasible path done")

            path_poses = path_poses_shortened
        else:
            # Creates a empty path message, which is by consent filled with, and only with the starting_point
            path_poses.append(self._get_pose_stamped(start_point, start_point))
            rospy.logerr("PathProvider: No possible path was found")

        # save current path_poses
        self.path_poses = path_poses

        # create self.path messages
        self._create_path_message(path_poses, debug)

    def _modify_lanelet(self, lanelet_id: int, modify_point: Point, start_point: Point):
        """Splits a lanelet at a certain point

        :param lanelet_id: lanelet to be split
        :param modify_point: point of split
        :return: ids of the split lanelet
        """
        # create new ids
        id_lane_1 = self._generate_lanelet_id(id_start=lanelet_id)
        id_lane_2 = self._generate_lanelet_id(id_start=lanelet_id, exclude=id_lane_1)
        # make a local copy of the lanelet to be removed
        lanelet_copy = self.map.lanelet_network.find_lanelet_by_id(lanelet_id)
        # bounds lanelet1
        sep_index = 0
        lanelet_center_list = lanelet_copy.center_vertices.tolist()
        end_index = self._find_nearest_path_index(lanelet_center_list, modify_point, use_posestamped=False)
        start_index = self._find_nearest_path_index(lanelet_center_list, start_point, use_posestamped=False)
        if end_index > start_index:
            sep_index = end_index - (abs(end_index - start_index) // 2)
        else:
            sep_index = end_index + (abs(end_index - start_index) // 2)
        if sep_index > 1 and len(lanelet_center_list) - sep_index > 1:

            # delete lanelet
            del self.map.lanelet_network._lanelets[lanelet_id]

            left_1 = lanelet_copy.left_vertices[:sep_index + 1]
            center_1 = lanelet_copy.center_vertices[:sep_index + 1]
            right_1 = lanelet_copy.right_vertices[:sep_index + 1]
            # bounds lanelet2
            left_2 = lanelet_copy.left_vertices[sep_index:]
            center_2 = lanelet_copy.center_vertices[sep_index:]
            right_2 = lanelet_copy.right_vertices[sep_index:]
            # create new lanelets lanelet_1 in front of obstacle; lanelet_2 behind obstacle
            lanelet_1 = Lanelet(lanelet_id=id_lane_1, predecessor=lanelet_copy.predecessor,
                                left_vertices=left_1, center_vertices=center_1, right_vertices=right_1,
                                successor=[id_lane_2], adjacent_left=lanelet_copy.adj_left,
                                adjacent_right=lanelet_copy.adj_right,
                                adjacent_right_same_direction=lanelet_copy.adj_right_same_direction,
                                adjacent_left_same_direction=lanelet_copy.adj_left_same_direction,
                                line_marking_left_vertices=lanelet_copy.line_marking_left_vertices,
                                line_marking_right_vertices=lanelet_copy.line_marking_right_vertices,
                                stop_line=lanelet_copy.stop_line,
                                lanelet_type=lanelet_copy.lanelet_type,
                                user_one_way=lanelet_copy.user_one_way,
                                user_bidirectional=lanelet_copy.user_bidirectional,
                                traffic_signs=lanelet_copy.traffic_signs,
                                traffic_lights=lanelet_copy.traffic_lights)

            lanelet_2 = Lanelet(lanelet_id=id_lane_2, predecessor=[id_lane_1],
                                left_vertices=left_2, center_vertices=center_2, right_vertices=right_2,
                                successor=lanelet_copy.successor, adjacent_left=lanelet_copy.adj_left,
                                adjacent_right=lanelet_copy.adj_right,
                                adjacent_right_same_direction=lanelet_copy.adj_right_same_direction,
                                adjacent_left_same_direction=lanelet_copy.adj_left_same_direction,
                                line_marking_left_vertices=lanelet_copy.line_marking_left_vertices,
                                line_marking_right_vertices=lanelet_copy.line_marking_right_vertices,
                                stop_line=lanelet_copy.stop_line,
                                lanelet_type=lanelet_copy.lanelet_type,
                                user_one_way=lanelet_copy.user_one_way,
                                user_bidirectional=lanelet_copy.user_bidirectional,
                                traffic_signs=lanelet_copy.traffic_signs,
                                traffic_lights=lanelet_copy.traffic_lights)
            # update predecessor and successor of surrounding prev/next lanes
            for succ in lanelet_copy.successor:
                self.map.lanelet_network.find_lanelet_by_id(succ)._predecessor.append(id_lane_2)
            for pred in lanelet_copy.predecessor:
                self.map.lanelet_network.find_lanelet_by_id(pred)._successor.append(id_lane_1)

            # update neigbourhood
            self._add_to_neighbourhood(id_lane_1, list([[id_lane_2], lanelet_copy.predecessor]))
            self._add_to_neighbourhood(id_lane_2, list([lanelet_copy.successor, [id_lane_1]]))
            # then add "back" to the lanelet_network
            self.map.lanelet_network.add_lanelet(lanelet_1)
            self.map.lanelet_network.add_lanelet(lanelet_2)
            # clean references
            self._fast_reference_cleanup(lanelet_id)
            return id_lane_1, id_lane_2
        return None, None

    def _generate_lanelet_id(self, id_start=-1, exclude: int = -1) -> int:
        """
        generates new unique lanelet id
        :param id_start: starting point for id selection
        :param exclude: id that should not be used
        :return: new unique lanelet id
        """
        while True:
            lane_id = 0
            if id_start == -1:
                lane_id = np.random.randint(0, sys.maxsize)
            else:
                lane_id = np.random.randint(id_start, sys.maxsize)
            if self.map.lanelet_network.find_lanelet_by_id(lane_id) is None and lane_id is not exclude:
                return lane_id

    def _add_to_neighbourhood(self, lanelet_id: int, entry: list):
        if len(entry) != 2:
            rospy.logerr("PathSupervisor: invalid neighbourhood update!")
        self.neighbourhood[lanelet_id] = entry

    def _analyze_neighbourhood(self, scenario_map: Scenario):
        """
        Analyze the neighbourhood of the given lanelet_network stored in the given map
        Therefore creating a dict, which stores for every lanelet all lanelets that know their relationship
        to the original lanelet
        :param scenario_map: commonroads scenario file
        """
        # init reference dict
        neighbourhood = {}
        for lane in scenario_map.lanelet_network.lanelets:
            entry = list()
            entry.append(list())  # successor list
            entry.append(list())  # predecessor list
            neighbourhood[lane.lanelet_id] = entry
        # document references
        for lane in scenario_map.lanelet_network.lanelets:
            for entry in lane.successor:
                neighbourhood[lane.lanelet_id][0].append(entry)
            for entry in lane.predecessor:
                neighbourhood[lane.lanelet_id][1].append(entry)

        return neighbourhood

    def _fast_reference_cleanup(self, lanelet_id: int):
        rospy.loginfo("PathSupervisor: Removing {} from references!".format(lanelet_id))
        # remove lanelet_id from all successors
        for succ in self.neighbourhood[lanelet_id][0]:
            self.map.lanelet_network.find_lanelet_by_id(succ)._predecessor.remove(lanelet_id)
        for pred in self.neighbourhood[lanelet_id][1]:
            self.map.lanelet_network.find_lanelet_by_id(pred)._successor.remove(lanelet_id)
        # remove lanelet_id from neighbourhood dict
        del self.neighbourhood[lanelet_id]

    def _update_network(self, matching_lanelet_id: int, modify_point: Point, start_point: Point, static_obstacle):
        """
        Splits a lanelet and it's neighbouring lanelets in half at a certain point, updates the reference graph
        and optionally adds a obstacle to the split lanelet

        :param matching_lanelet_id: lanelet to be split
        :param modify_point: point of split
        :param static_obstacle: obstacle to be added
        :return: ids of the split lanelet
        """
        # new neighbourhood
        right_1 = None
        right_2 = None
        left_1 = None
        left_2 = None
        # also split neighbours
        if self.map.lanelet_network.find_lanelet_by_id(matching_lanelet_id).adj_right is not None:
            right_1, right_2 = self._modify_lanelet(
                self.map.lanelet_network.find_lanelet_by_id(matching_lanelet_id).adj_right,
                modify_point, start_point)
        if self.map.lanelet_network.find_lanelet_by_id(matching_lanelet_id).adj_left is not None:
            left_1, left_2 = self._modify_lanelet(
                self.map.lanelet_network.find_lanelet_by_id(matching_lanelet_id).adj_left,
                modify_point, start_point)

        # split obstacle lanelet
        matching_1, matching_2 = self._modify_lanelet(matching_lanelet_id, modify_point, start_point)

        # update neighbourhood
        if right_1 is not None:
            # update right side of current lanelet
            self.map.lanelet_network.find_lanelet_by_id(matching_1)._adj_right = right_1
            self.map.lanelet_network.find_lanelet_by_id(matching_2)._adj_right = right_2
            next = None
            next_dir = None
            if self.map.lanelet_network.find_lanelet_by_id(matching_1).adj_right_same_direction:
                next = self.map.lanelet_network.find_lanelet_by_id(right_1)._adj_right
                self.map.lanelet_network.find_lanelet_by_id(right_1)._adj_left = matching_1
                self.map.lanelet_network.find_lanelet_by_id(right_2)._adj_left = matching_2
                next_dir = True
            else:
                next = self.map.lanelet_network.find_lanelet_by_id(right_1)._adj_left
                self.map.lanelet_network.find_lanelet_by_id(right_2)._adj_right = matching_2
                self.map.lanelet_network.find_lanelet_by_id(right_1)._adj_right = matching_1
                next_dir = False

            # make sure that the changes propagate through, but only in the same direction
            while next is not None:
                right_1_old = right_1
                right_2_old = right_2
                right_1, right_2 = self._modify_lanelet(next, modify_point, start_point)
                if next_dir:
                    self.map.lanelet_network.find_lanelet_by_id(right_1_old)._adj_right = right_1
                    self.map.lanelet_network.find_lanelet_by_id(right_2_old)._adj_right = right_2
                    next_dir = self.map.lanelet_network.find_lanelet_by_id(right_1_old)._adj_right_same_direction
                else:
                    self.map.lanelet_network.find_lanelet_by_id(right_1_old)._adj_left = right_1
                    self.map.lanelet_network.find_lanelet_by_id(right_2_old)._adj_left = right_2
                    next_dir = self.map.lanelet_network.find_lanelet_by_id(right_1_old)._adj_left_same_direction

                if next_dir:
                    self.map.lanelet_network.find_lanelet_by_id(right_1)._adj_left = right_1_old
                    self.map.lanelet_network.find_lanelet_by_id(right_2)._adj_left = right_2_old
                    next =self.map.lanelet_network.find_lanelet_by_id(right_1)._adj_right
                else:
                    self.map.lanelet_network.find_lanelet_by_id(right_1)._adj_right = right_1_old
                    self.map.lanelet_network.find_lanelet_by_id(right_2)._adj_right = right_2_old
                    next =self.map.lanelet_network.find_lanelet_by_id(right_1)._adj_left

        if left_1 is not None:
            # update left side of current lanelet
            self.map.lanelet_network.find_lanelet_by_id(matching_1)._adj_left = left_1
            self.map.lanelet_network.find_lanelet_by_id(matching_2)._adj_left = left_2
            next = None
            next_dir = None
            if self.map.lanelet_network.find_lanelet_by_id(matching_1).adj_left_same_direction:
                next = self.map.lanelet_network.find_lanelet_by_id(left_1)._adj_left
                self.map.lanelet_network.find_lanelet_by_id(left_1)._adj_right = matching_1
                self.map.lanelet_network.find_lanelet_by_id(left_2)._adj_right = matching_2
                next_dir = True
            else:
                next = self.map.lanelet_network.find_lanelet_by_id(left_1)._adj_right
                self.map.lanelet_network.find_lanelet_by_id(left_2)._adj_left = matching_2
                self.map.lanelet_network.find_lanelet_by_id(left_1)._adj_left = matching_1
                next_dir = False

            # make sure that the changes propagate through, but only in the same direction
            while next is not None:
                left_1_old = left_1
                left_2_old = left_2
                left_1, left_2 = self._modify_lanelet(next, modify_point, start_point)
                if next_dir:
                    self.map.lanelet_network.find_lanelet_by_id(left_1_old)._adj_left = left_1
                    self.map.lanelet_network.find_lanelet_by_id(left_2_old)._adj_left = left_2
                    next_dir = self.map.lanelet_network.find_lanelet_by_id(left_1_old)._adj_left_same_direction
                else:
                    self.map.lanelet_network.find_lanelet_by_id(left_1_old)._adj_right = left_1
                    self.map.lanelet_network.find_lanelet_by_id(left_2_old)._adj_right = left_2
                    next_dir = self.map.lanelet_network.find_lanelet_by_id(left_1_old)._adj_right_same_direction

                if next_dir:
                    self.map.lanelet_network.find_lanelet_by_id(left_1)._adj_right = left_1_old
                    self.map.lanelet_network.find_lanelet_by_id(left_2)._adj_right = left_2_old
                    next =self.map.lanelet_network.find_lanelet_by_id(left_1)._adj_left
                else:
                    self.map.lanelet_network.find_lanelet_by_id(left_1)._adj_left = left_1_old
                    self.map.lanelet_network.find_lanelet_by_id(left_2)._adj_left = left_2_old
                    next =self.map.lanelet_network.find_lanelet_by_id(left_1)._adj_right

        # add obstacle
        if static_obstacle is not None:
            self.map.lanelet_network.find_lanelet_by_id(matching_2).add_static_obstacle_to_lanelet(
                static_obstacle.obstacle_id)
            self.map.add_objects(static_obstacle)
        return matching_1, matching_2
