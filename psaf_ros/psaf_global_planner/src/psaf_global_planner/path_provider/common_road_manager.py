#!/usr/bin/env python
import string

from commonroad.scenario.scenario import Scenario
from commonroad.scenario.lanelet import Lanelet
from copy import deepcopy

from geometry_msgs.msg import Point
from psaf_messages.msg import XLanelet, CenterLineExtended
from commonroad.scenario.traffic_sign import TrafficSignIDGermany, TrafficLight, TrafficSign, TrafficSignElement
from typing import *

import numpy as np
import rospy


class CommonRoadManager:

    def __init__(self, hd_map: Scenario, map_name: string, default_speed: int = 50, intersections: Dict = {}):
        rospy.loginfo("CommonRoadManager: Started!")
        self.default_speed = default_speed
        self.map = hd_map
        # create neighbourhood dicts for efficient access to that information
        self.neighbourhood = self._analyze_neighbourhood(self.map)
        self.original_neighbourhood = deepcopy(self.neighbourhood)
        self.message_by_lanelet = {}
        self.intersections = deepcopy(intersections)
        self._fill_message_dict()
        self._handle_turnaround_town_03(map_name)
        self.original_map = deepcopy(self.map)
        self.original_message_by_lanelet = deepcopy(self.message_by_lanelet)
        rospy.loginfo("CommonRoadManager: Done!")

    def _handle_turnaround_town_03(self, map_name: string):
        if map_name == "Town03":
            rospy.loginfo("Handling turnaround")
            # coordinates for turnaround splits
            split_point: list = [[6, -50], [-6, -50], [50, -6]]
            # handling split to optimize the turnaround
            for listpos, point in enumerate(split_point):
                matching_lanelet_id = self.map.lanelet_network.lanelets_in_proximity(np.array(point), 10)
                # remove the lanelet, to be split, from the intersections dict
                self.intersections.pop(matching_lanelet_id[0].lanelet_id)

                split_1, split_2 = self.update_network(matching_lanelet_id=matching_lanelet_id[0].lanelet_id,
                                                       modify_point=Point(x=point[0], y=point[1]),
                                                       start_point=Point(x=point[0], y=point[1]), static_obstacle=None)
                # add the new lanelet to the intersections dict
                self.intersections[split_1] = False
                self.intersections[split_2] = True
                # add traffic signs
                if listpos == 0:
                    if self.map.lanelet_network.find_lanelet_by_id(split_1).adj_right_same_direction and self.map.lanelet_network.find_lanelet_by_id(split_1).adj_right is not None:
                        self._add_sign_to_lanelet(lanelet_id=self.map.lanelet_network.find_lanelet_by_id(split_1).adj_right,
                                                  pos_index=len(self.map.lanelet_network.find_lanelet_by_id(
                                                      split_1).center_vertices) - 1,
                                                  typ=TrafficSignIDGermany.STOP,cur_mark_id =split_1+1)
                        self.message_by_lanelet[self.map.lanelet_network.find_lanelet_by_id(split_1).adj_right].hasStop = True
                    if self.map.lanelet_network.find_lanelet_by_id(split_1).adj_left_same_direction and self.map.lanelet_network.find_lanelet_by_id(split_1).adj_left is not None:
                        self._add_sign_to_lanelet(lanelet_id=self.map.lanelet_network.find_lanelet_by_id(split_1).adj_left,
                                                  pos_index=len(self.map.lanelet_network.find_lanelet_by_id(
                                                      split_1).center_vertices) - 1,
                                                  typ=TrafficSignIDGermany.STOP,cur_mark_id =split_1-1)
                        self.message_by_lanelet[self.map.lanelet_network.find_lanelet_by_id(split_1).adj_left].hasStop = True
                    self._add_sign_to_lanelet(lanelet_id=split_1,
                                              pos_index=len(self.map.lanelet_network.find_lanelet_by_id(split_1).center_vertices)-1,
                                              typ=TrafficSignIDGermany.STOP, cur_mark_id =split_1)
                    self.message_by_lanelet[split_1].hasStop = True
            # add stop signs to not split lanes
            not_split: list = [347, 281, 277, 181, 184]
            for _id in not_split:
                self._add_sign_to_lanelet(lanelet_id=_id,
                                          pos_index=len(self.map.lanelet_network.find_lanelet_by_id(_id).center_vertices)-1,
                                          typ=TrafficSignIDGermany.STOP, cur_mark_id =_id)

    def _add_sign_to_lanelet(self, lanelet_id: int, pos_index: int, typ: TrafficSignIDGermany, additional: list = [],
                             cur_mark_id = -1):
        pos = self.map.lanelet_network.find_lanelet_by_id(lanelet_id).center_vertices[pos_index]
        sign_element = TrafficSignElement(typ, additional)
        id_set = set()
        id_set.add(lanelet_id)
        sign = TrafficSign(cur_mark_id, first_occurrence=deepcopy(id_set), position=pos,
                           traffic_sign_elements=[sign_element])
        self.map.lanelet_network.add_traffic_sign(sign, lanelet_ids=deepcopy(id_set))

    def _update_message_dict(self, matching_lanelet_id: int, lanelet_front: int, lanelet_back: int):
        # add the new lanelets to the dict
        self._generate_xlanelet(self.map.lanelet_network.find_lanelet_by_id(lanelet_front))
        self._generate_xlanelet(self.map.lanelet_network.find_lanelet_by_id(lanelet_back))

        # remove old lanelet from dict
        del self.message_by_lanelet[matching_lanelet_id]

    def _fill_message_dict(self):
        rospy.loginfo("CommonRoadManager: Message calculation started!")
        for lanelet in self.map.lanelet_network.lanelets:
            self._generate_xlanelet(lanelet)
        rospy.loginfo("CommonRoadManager: Message calculation done!")

    def _calculate_distance_and_duration(self, previous: list, current: list, prev_speed: float):
        # calculate the distance between two waypoints
        distance = np.linalg.norm(np.array(current) - np.array(previous))
        # calculate time by distance [m] and speed [km/h] -> transform to [m/s]
        time_spent = distance / (prev_speed / 3.6)

        return distance, time_spent

    def _generate_xlanelet(self, lanelet: Lanelet):
        stop = False
        for sign_id in lanelet.traffic_signs:
            sign = self.map.lanelet_network.find_traffic_sign_by_id(sign_id)
            if sign.traffic_sign_elements[0].traffic_sign_element_id == TrafficSignIDGermany.STOP:
                stop = True

        light = len(lanelet.traffic_lights) > 0
        intersection = self._check_in_lanelet_for_intersection(lanelet)
        center_line = self._generate_extended_centerline_by_lanelet(lanelet)

        # create message, isLaneChange is False by default
        message = XLanelet(id=lanelet.lanelet_id, hasLight=light, isAtIntersection=intersection, hasStop=stop,
                           isLaneChange=False, route_portion=center_line)

        self.message_by_lanelet[lanelet.lanelet_id] = message

    def _generate_extended_centerline_by_lanelet(self, lanelet: Lanelet) -> List[CenterLineExtended]:
        """
        Generate the extended centerline Message based on the given lanelet.
        Therefore check the current speed limit for every waypoint.
        :param lanelet: lanelet of which the given extended centerline should be generated
        :return: List of Waypoints and their corresponding speed. -> [[x,y,z, speed], ..]
        """
        from psaf_global_planner.path_provider.path_provider_common_roads import PathProviderCommonRoads as pp
        # first get speed_signs in current lanelet
        speed_signs = []
        for sign_id in lanelet.traffic_signs:
            sign = self.map.lanelet_network.find_traffic_sign_by_id(sign_id)
            if sign.traffic_sign_elements[0].traffic_sign_element_id == TrafficSignIDGermany.MAX_SPEED:
                speed = int(sign.traffic_sign_elements[0].additional_values[0])
                pos = Point(sign.position[0], sign.position[1], 0)
                index = pp.find_nearest_path_index(lanelet.center_vertices, pos, use_xcenterline=False)
                speed_signs.append([index, speed])

        # sort speed signs to ease location based access
        speed_signs.sort(key=lambda x: x[0])

        # generate CenterLineExtended
        center_line_extended = []
        speed = self.default_speed
        time = 0.0
        dist = 0.0
        prev_point = None
        for i, point in enumerate(lanelet.center_vertices):
            if i > 0:
                tmp_dist, tmp_time = self._calculate_distance_and_duration(previous=prev_point, current=point,
                                                                           prev_speed=speed)
                dist += tmp_dist
                time += tmp_time

            # check for speed signs
            for speed_sign in speed_signs:
                if i >= speed_sign[0]:
                    speed = speed_sign[1]
                else:
                    break

            prev_point = point
            # fill center_line_extended
            center_line_extended.append(
                CenterLineExtended(x=point[0], y=point[1], z=0, speed=speed, duration=time, distance=dist))

        return center_line_extended

    def _check_in_lanelet_for_intersection(self, lanelet: Lanelet) -> bool:
        """
        Checks whether a lanelet is on an intersection
        :param lanelet: lanelet to be checked
        :return: True if lanelet is on an intersection, False if not
        """

        # check for the amount of predecessor
        if lanelet.lanelet_id in self.intersections:
            return self.intersections[lanelet.lanelet_id]
        else:
            return False

    def _modify_lanelet(self, lanelet_id: int, modify_point: Point, start_point: Point) -> Tuple[int, int]:
        from psaf_global_planner.path_provider.path_provider_common_roads import PathProviderCommonRoads as pp
        """Splits a lanelet at a certain point

        :param lanelet_id: lanelet to be split
        :param modify_point: point of split
        :return: ids of the split lanelet
        """
        # create new ids
        id_lane_1 = self._generate_lanelet_id(id_start=lanelet_id)
        id_lane_2 = self._generate_lanelet_id(id_start=lanelet_id, exclude=id_lane_1)
        # make a local copy of the lanelet to be removed
        # if a lanelet can't be found -> exit
        lanelet_copy = self.map.lanelet_network.find_lanelet_by_id(lanelet_id)
        if lanelet_copy is None:
            return None, None
        # bounds lanelet1
        sep_index = 0
        lanelet_center_list = lanelet_copy.center_vertices.tolist()
        end_index = pp.find_nearest_path_index(lanelet_center_list, modify_point, use_xcenterline=False)
        start_index = pp.find_nearest_path_index(lanelet_center_list, start_point, use_xcenterline=False)
        if end_index > start_index:
            sep_index = end_index - (abs(end_index - start_index) // 2)
        else:
            sep_index = end_index + (abs(end_index - start_index) // 2)
        if sep_index > 1 and len(lanelet_center_list) - sep_index > 1:
            # bound lanelet_1
            left_1 = lanelet_copy.left_vertices[:sep_index + 1]
            center_1 = lanelet_copy.center_vertices[:sep_index + 1]
            right_1 = lanelet_copy.right_vertices[:sep_index + 1]

            # bounds lanelet_2
            left_2 = lanelet_copy.left_vertices[sep_index:]
            center_2 = lanelet_copy.center_vertices[sep_index:]
            right_2 = lanelet_copy.right_vertices[sep_index:]
            # add traffic signs to lanelet_2 / lanelet_1
            signs_1 = set()
            signs_2 = set()
            for sign_id in lanelet_copy.traffic_signs:
                sign = self.map.lanelet_network.find_traffic_sign_by_id(sign_id)
                pos = Point(sign.position[0], sign.position[1], 0)
                index = pp.find_nearest_path_index(lanelet_copy.center_vertices, pos, use_xcenterline=False)
                # remove old reference
                sign._first_occurrence.remove(lanelet_copy.lanelet_id)
                # determine whether the sign should be put on lane 1 or 2
                if index < sep_index:
                    signs_1.add(sign_id)
                    sign._first_occurrence.add(id_lane_1)
                    signs_2.add(sign_id)
                    sign._first_occurrence.add(id_lane_2)
                else:
                    signs_2.add(sign_id)
                    sign.first_occurrence.add(id_lane_2)
            # delete lanelet
            del self.map.lanelet_network._lanelets[lanelet_id]
            # create new lanelets: lanelet_2 lane without obstacle; lanelet_1 lane with obstacle
            lanelet_1 = Lanelet(lanelet_id=id_lane_1, predecessor=lanelet_copy.predecessor,
                                left_vertices=left_1, center_vertices=center_1, right_vertices=right_1,
                                successor=[id_lane_2], adjacent_left=lanelet_copy.adj_left,
                                adjacent_right=lanelet_copy.adj_right,
                                adjacent_right_same_direction=lanelet_copy.adj_right_same_direction,
                                adjacent_left_same_direction=lanelet_copy.adj_left_same_direction,
                                line_marking_left_vertices=lanelet_copy.line_marking_left_vertices,
                                line_marking_right_vertices=lanelet_copy.line_marking_right_vertices,
                                stop_line=None,
                                lanelet_type=lanelet_copy.lanelet_type,
                                user_one_way=lanelet_copy.user_one_way,
                                user_bidirectional=lanelet_copy.user_bidirectional,
                                traffic_signs=signs_1,
                                traffic_lights=None)

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
                                traffic_signs=signs_2,
                                traffic_lights=lanelet_copy.traffic_lights)
            # update predecessor and successor of surrounding prev/next lanes
            for succ in lanelet_copy.successor:
                # if a lanelet can't be found -> exit
                if self.map.lanelet_network.find_lanelet_by_id(succ) is None:
                    continue
                self.map.lanelet_network.find_lanelet_by_id(succ)._predecessor.append(id_lane_2)
            for pred in lanelet_copy.predecessor:
                # if a lanelet can't be found -> exit
                if self.map.lanelet_network.find_lanelet_by_id(pred) is None:
                    continue
                self.map.lanelet_network.find_lanelet_by_id(pred)._successor.append(id_lane_1)
            # update neigbourhood
            self._add_to_neighbourhood(id_lane_1, list([[id_lane_2], lanelet_copy.predecessor]))
            self._add_to_neighbourhood(id_lane_2, list([lanelet_copy.successor, [id_lane_1]]))
            # then add "back" to the lanelet_network
            self.map.lanelet_network.add_lanelet(lanelet_1)
            self.map.lanelet_network.add_lanelet(lanelet_2)
            # clean references
            self._fast_reference_cleanup(lanelet_id)
            self._update_message_dict(lanelet_copy.lanelet_id, id_lane_1, id_lane_2)
            return id_lane_1, id_lane_2
        return None, None

    def _generate_lanelet_id(self, id_start=-1, exclude: int = -1) -> int:
        """
        generates new unique lanelet id
        :param id_start: starting point for id selection
        :param exclude: id that should not be used
        :return: new unique lanelet id
        """
        max_uint16 = 2 ** 16
        while True:
            lane_id = 0
            if id_start == -1:
                lane_id = np.random.randint(0, max_uint16)
            else:
                lane_id = np.random.randint(id_start, max_uint16)
            if self.map.lanelet_network.find_lanelet_by_id(lane_id) is None and lane_id != exclude:
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
        # if a lanelet can't be found -> exit
        if self.map.lanelet_network.find_lanelet_by_id(matching_lanelet_id) is None:
            return None, None
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
        if matching_1 is None or matching_2 is None:
            return None, None

        # update neighbourhood
        if right_1 is not None and right_2 is not None:
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
                if right_1 is None or right_2 is None:
                    return None, None
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

        if left_1 is not None and left_2 is not None:
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
                if left_1 is None or left_2 is None:
                    return None, None
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
            if matching_2 is not None:
                self.map.lanelet_network.find_lanelet_by_id(matching_2).add_static_obstacle_to_lanelet(
                    static_obstacle.obstacle_id)
            else:
                self.map.lanelet_network.find_lanelet_by_id(matching_lanelet_id).add_static_obstacle_to_lanelet(
                    static_obstacle.obstacle_id)
            self.map.add_objects(static_obstacle)
        return matching_1, matching_2