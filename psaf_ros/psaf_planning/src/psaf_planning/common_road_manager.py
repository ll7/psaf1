#!/usr/bin/env python

from commonroad.scenario.scenario import Scenario
from commonroad.scenario.lanelet import Lanelet
from copy import deepcopy
from geometry_msgs.msg import Point
from psaf_messages.msg import XLanelet, CenterLineExtended
from commonroad.scenario.traffic_sign import TrafficSignIDGermany, TrafficLight
from typing import *

import numpy as np
import rospy
import sys

class CommonRoadManager:

    def __init__(self, hd_map: Scenario, default_speed: int = 50):
        rospy.loginfo("CommonRoadManager: Started!")
        self.default_speed = default_speed
        self.map = hd_map
        self.original_map = deepcopy(self.map)
        # create neighbourhood dicts for efficient access to that information
        self.neighbourhood = self._analyze_neighbourhood(self.original_map)
        self.original_neighbourhood = deepcopy(self.neighbourhood)
        self.message_by_lanelet = {}

        # test scenario
        pos = len(self.original_map.lanelet_network.find_lanelet_by_id(100).center_vertices) // 2
        pos = self.original_map.lanelet_network.find_lanelet_by_id(100).center_vertices[pos]
        traffic_light = TrafficLight(1, [], pos)
        id_set = set()
        id_set.add(100)
        self.original_map.lanelet_network.add_traffic_light(traffic_light, lanelet_ids=id_set)

        self._fill_message_dict()

    def _fill_message_dict(self):
        rospy.loginfo("CommonRoadManager: Message Hashmap calculation started!")

        for lanelet in self.original_map.lanelet_network.lanelets:
            stop = False
            for sign_id in lanelet.traffic_signs:
                sign = self.original_map.lanelet_network.find_traffic_sign_by_id(sign_id)
                if sign.traffic_sign_elements[0].traffic_sign_element_id == TrafficSignIDGermany.STOP:
                    stop = True

            light = len(lanelet.traffic_lights) > 0
            intersection = self._check_in_lanelet_for_intersection(lanelet)
            center_line = self._generate_extended_centerline_by_lanelet(lanelet)

            # create message
            message = XLanelet(id=lanelet.lanelet_id, hasLight=light, isAtIntersection=intersection, hasStop=stop,
                               route_portion=center_line)

            self.message_by_lanelet[lanelet.lanelet_id] = message
        rospy.loginfo("CommonRoadManager: Message Hashmap calculation done!")

    def _generate_extended_centerline_by_lanelet(self, lanelet: Lanelet) -> List[CenterLineExtended]:
        """
        Generate the extended centerline Message based on the given lanelet.
        Therefore check the current speed limit for every waypoint.
        :param lanelet: lanelet of which the given extended centerline should be generated
        :return: List of Waypoints and their corresponding speed. -> [[x,y,z, speed], ..]
        """
        from psaf_planning.global_planner.path_provider_common_roads import PathProviderCommonRoads as pp
        # first get speed_signs in current lanelet
        speed_signs = []
        for sign_id in lanelet.traffic_signs:
            sign = self.original_map.lanelet_network.find_traffic_sign_by_id(sign_id)
            if sign.traffic_sign_elements[0].traffic_sign_element_id == TrafficSignIDGermany.MAX_SPEED:
                speed = int(sign.traffic_sign_elements[0].additional_values[0])
                pos = Point(sign.position[0], sign.position[1], 0)
                index = pp.find_nearest_path_index(lanelet.center_vertices, pos, use_posestamped=False)
                speed_signs.append([index, speed])

        # sort speed signs to ease location based access
        speed_signs.sort(key=lambda x: x[0])

        # generate CenterLineExtended
        center_line_extended = []
        for i, point in enumerate(lanelet.center_vertices):
            # check for speed signs
            speed = self.default_speed
            for speed_sign in speed_signs:
                if i >= speed_sign[0]:
                    speed = speed_sign[1]

            # fill center_line_extended
            center_line_extended.append(CenterLineExtended(x=point[0], y=point[1], z=0, speed=speed))

        return center_line_extended

    def _check_in_lanelet_for_intersection(self, lanelet: Lanelet) -> bool:
        """
        Checks whether a intersection is ahead of the given lanelet
        :param lanelet: lanelet to be checked
        :return: True if there is a upcoming intersection, False if not
        """
        # get the successor of the successor of the given lanelet
        lane = lanelet
        for i in range(0, 2):
            if len(lane.successor) == 0:
                return False
            succ = lanelet.successor[0]
            lane = self.original_map.lanelet_network.find_lanelet_by_id(succ)

        # check for the amount of predecessor
        return len(lane.predecessor) > 1

    def _modify_lanelet(self, lanelet_id: int, modify_point: Point, start_point: Point) -> Tuple[int, int]:
        from psaf_planning.global_planner.path_provider_common_roads import PathProviderCommonRoads as pp
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
        end_index = pp.find_nearest_path_index(lanelet_center_list, modify_point, use_posestamped=False)
        start_index = pp.find_nearest_path_index(lanelet_center_list, start_point, use_posestamped=False)
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

    def _analyze_neighbourhood(self, scenario_map: Scenario) -> Dict:
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
            entry.append(list())  # successor alist
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

    def update_network(self, matching_lanelet_id: int, modify_point: Point, start_point: Point, static_obstacle) -> \
            Tuple[int, int]:
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
                    next = self.map.lanelet_network.find_lanelet_by_id(right_1)._adj_right
                else:
                    self.map.lanelet_network.find_lanelet_by_id(right_1)._adj_right = right_1_old
                    self.map.lanelet_network.find_lanelet_by_id(right_2)._adj_right = right_2_old
                    next = self.map.lanelet_network.find_lanelet_by_id(right_1)._adj_left

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
                    next = self.map.lanelet_network.find_lanelet_by_id(left_1)._adj_left
                else:
                    self.map.lanelet_network.find_lanelet_by_id(left_1)._adj_left = left_1_old
                    self.map.lanelet_network.find_lanelet_by_id(left_2)._adj_left = left_2_old
                    next = self.map.lanelet_network.find_lanelet_by_id(left_1)._adj_right

        # add obstacle
        if static_obstacle is not None:
            self.map.lanelet_network.find_lanelet_by_id(matching_2).add_static_obstacle_to_lanelet(
                static_obstacle.obstacle_id)
            self.map.add_objects(static_obstacle)
        return matching_1, matching_2