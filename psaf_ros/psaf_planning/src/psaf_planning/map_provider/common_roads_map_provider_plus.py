# !/usr/bin/env python
import tempfile

import rospy
from carla_msgs.msg import CarlaWorldInfo
from commonroad.common.file_writer import CommonRoadFileWriter
from commonroad.scenario.trajectory import State
from commonroad.planning.goal import GoalRegion
from commonroad.common.util import Interval
from commonroad.geometry.shape import Circle
from commonroad.common.file_writer import CommonRoadFileWriter
from commonroad.common.file_writer import OverwriteExistingFile
from commonroad.scenario.scenario import Location
from commonroad.scenario.scenario import Tag

from psaf_planning.map_provider.landmark_provider import LandMarkProvider, LandMarkPoint
from psaf_planning.map_provider.map_provider import MapProvider
from geometry_msgs.msg import Point
from commonroad.scenario.scenario import Scenario
from commonroad.scenario.lanelet import Lanelet
from commonroad.scenario.traffic_sign import TrafficSignIDGermany, TrafficLight, TrafficSign, TrafficSignElement
from copy import deepcopy
import math

import numpy as np
import matplotlib.pyplot as plt

from commonroad.planning.planning_problem import PlanningProblem, PlanningProblemSet
from commonroad.visualization.draw_dispatch_cr import draw_object


class CommonRoadMapProvider(MapProvider):

    def __init__(self, init_rospy: bool = False):
        if init_rospy:
            rospy.init_node('CommonRoadMapProvider', anonymous=True)
        super(CommonRoadMapProvider, self).__init__()
        self.intersection = {}
        self.map_cr: Scenario = None
        self.landmark_provider: LandMarkProvider = None
        self.cur_light_id = -1
        self.planning_problem = None

        # orientation difference between traffic light and lanelet in degree,
        # up to which the a lanelet and a traffic light is considered a match
        self.max_angle_diff = 20  # in degree

        # length and orientation differences
        # up to which two lanelets are considered as neighbouring lanelets
        self.max_length_diff = 0.1  # in percent
        self.max_neighbour_angle_diff = 10  # in degree

        # define the standard intersection width, which is used to search for neighbouring lanelets in intersections
        self.inter_width = 15  # in meter

    def update_world(self, world_info: CarlaWorldInfo):
        """
        Check if a new map was sent and receive it
        """
        self.map_ready = False
        rospy.loginfo("CommonRoadMapProvider: Received new map info")
        if self.map_name == world_info.map_name:
            rospy.loginfo("CommonRoadMapProvider: Map already loaded")
        else:
            self.cur_light_id = -1
            self.map_name = world_info.map_name
            self.map = world_info.opendrive
            self.map_ready = True
            rospy.loginfo("CommonRoadMapProvider: Received: " + self.map_name)
            self.map_cr = self.convert_od_to_lanelet()
            rospy.loginfo("CommonRoadMapProvider: Getting landmarks")
            self.landmark_provider = LandMarkProvider()
            rospy.loginfo("CommonRoadMapProvider: detecting intersections")
            self._detect_intersections()
            rospy.loginfo("CommonRoadMapProvider: Adding traffic lights")
            self._traffic_lights_to_scenario(self.landmark_provider.get_marks_by_categorie('Signal_3Light_Post01'))
            rospy.loginfo("CommonRoadMapProvider: DONE")
            self.planning_problem = self._generate_dummy_planning_problem()
            self._visualize_scenario(self.map_cr, self.planning_problem)

    def _find_nearest_lanlet(self, goal: Point):
        """
        Given a Point (x,y,z) -> find nearest lanelet
        :param goal: point to which the nearest lanelet should be searched
        :return: nearest lanelet to point goal
        """
        nearest = None
        curr_radius = 1
        step_size = 0.5
        max_radius = 100
        while curr_radius < max_radius or nearest is not None:
            nearest = self.map_cr.lanelet_network.lanelets_in_proximity(np.array([goal.x, goal.y]), curr_radius)
            if len(nearest) == 0:
                nearest = None
                curr_radius += step_size
            else:
                return nearest[0]
        return None

    def _find_traffic_light_lanelet(self, light: LandMarkPoint):
        """
        Given a traffic light -> find the lanelet to which the traffic light is effective
        :param light: trafficLight as a LandMarkPoint
        :return: lanelets to which the traffic light is effective
        """
        curr_radius = self.inter_width * 3
        step_size = self.inter_width
        max_radius = self.inter_width * 5
        solution_found = False
        solution_list = []
        nearest = []
        best_lanelets = []
        while not solution_found and curr_radius <= max_radius:
            nearest = self.map_cr.lanelet_network.lanelets_in_proximity(np.array([light.x, light.y]), curr_radius)
            if len(nearest) > 0:
                for i, lane in enumerate(nearest):
                    # get whether the lanelet is on an intersection
                    is_intersection = self.intersection[lane.lanelet_id]
                    # get orientation of that lane
                    lane_orientation = self._get_lanelet_orientation_to_light(lane, use_end=True)
                    angle_diff = 180 - abs(abs(light.orientation - lane_orientation) - 180)

                    if not is_intersection and angle_diff < self.max_angle_diff:
                        # get distance of the end point of that lanelet to the traffic light
                        dist_end = np.linalg.norm(np.array([light.x, light.y]) - np.array(lane.center_vertices[-1]))
                        # get distance of the start point of that lanelet to the traffic light
                        dist_start = np.linalg.norm(np.array([light.x, light.y]) - np.array(lane.center_vertices[0]))
                        if dist_start > dist_end:
                            solution_list.append((i, dist_end))
                            solution_found = True

                curr_radius += step_size

        if solution_found:
            best = min(solution_list, key=lambda x: x[1])
            best_lanelets = [nearest[best[0]]]

            neighbour_lanelets = best_lanelets[0]

            # get all right neighbours
            while neighbour_lanelets.adj_right_same_direction and neighbour_lanelets.adj_right is not None:
                neighbour_lanelets = self.map_cr.lanelet_network.find_lanelet_by_id(neighbour_lanelets.adj_right)
                best_lanelets.append(neighbour_lanelets)

            neighbour_lanelets = best_lanelets[0]
            # get all left neighbours
            while neighbour_lanelets.adj_left_same_direction and neighbour_lanelets.adj_left is not None:
                neighbour_lanelets = self.map_cr.lanelet_network.find_lanelet_by_id(neighbour_lanelets.adj_left)
                best_lanelets.append(neighbour_lanelets)

        return best_lanelets

    def _detect_intersections(self):
        """
        Detect all intersections on the map and store the results in an intersection hashmap with a lanelet_id as key
        """
        for lanelet in self.map_cr.lanelet_network.lanelets:
            # get the successor of the successor of the given lanelet
            lane = lanelet
            if len(lane.successor) == 0:
                return False
            succ = lanelet.successor[0]
            lane = self.map_cr.lanelet_network.find_lanelet_by_id(succ)

            # check for the amount of predecessor
            self.intersection[lanelet.lanelet_id] = len(lane.predecessor) > 1

        # now double check -> if my neighbour is at an intersection, i am too
        # to catch cases where a lanelet is not affected by an intersection at all
        for lanelet in self.map_cr.lanelet_network.lanelets:
            if self.intersection[lanelet.lanelet_id]:
                neighbour_lanelet = lanelet
                # go through all right neighbours
                while neighbour_lanelet.adj_right_same_direction and neighbour_lanelet.adj_right is not None:
                    neighbour_lanelet = self.map_cr.lanelet_network.find_lanelet_by_id(neighbour_lanelet.adj_right)
                    self.intersection[neighbour_lanelet.lanelet_id] = True

                neighbour_lanelet = lanelet
                # go through all left neighbours
                while neighbour_lanelet.adj_left_same_direction and neighbour_lanelet.adj_left is not None:
                    neighbour_lanelet = self.map_cr.lanelet_network.find_lanelet_by_id(neighbour_lanelet.adj_left)
                    self.intersection[neighbour_lanelet.lanelet_id] = True

        # following the upper issue: now catch cases where you dont know your neighbour lanelet
        for lanelet in self.map_cr.lanelet_network.lanelets:
            if self.intersection[lanelet.lanelet_id]:
                center_point = lanelet.center_vertices[len(lanelet.center_vertices) // 2]
                start_point = lanelet.center_vertices[0]
                end_point = lanelet.center_vertices[-1]
                length = np.linalg.norm(np.array(end_point) - np.array(start_point))

                start_orientation = self._get_lanelet_orientation_to_light(lanelet, use_end=False)
                end_orientation = self._get_lanelet_orientation_to_light(lanelet, use_end=True)

                # get all lanelets nearby
                nearest = self.map_cr.lanelet_network.lanelets_in_proximity(np.array(center_point), self.inter_width)
                for near_lanelet in nearest:
                    curr_start_point = near_lanelet.center_vertices[0]
                    curr_end_point = near_lanelet.center_vertices[-1]
                    curr_length = np.linalg.norm(np.array(curr_end_point) - np.array(curr_start_point))

                    curr_start_orientation = self._get_lanelet_orientation_to_light(near_lanelet, use_end=False)
                    curr_end_orientation = self._get_lanelet_orientation_to_light(near_lanelet, use_end=True)

                    # calculate angle_diff but consider that the two orientations should be opposite to each other
                    start_angle_diff = abs(abs(start_orientation - curr_end_orientation) - 180)
                    end_angle_diff = abs(abs(end_orientation - curr_start_orientation) - 180)

                    # do these two lanelets fulfill the neighbouring criteria (defined in init)
                    if start_angle_diff < self.max_neighbour_angle_diff and \
                            end_angle_diff < self.max_neighbour_angle_diff and \
                            math.isclose(length, curr_length, rel_tol=self.max_length_diff):
                        self.intersection[near_lanelet.lanelet_id] = True

    def _traffic_lights_to_scenario(self, lights: list):
        """
        Add the traffic lights to the scenario
        :param lights: list of trafficLights as LandMarkPoints
        """
        for light in lights:
            mapped_lanelets = self._find_traffic_light_lanelet(light)
            if len(mapped_lanelets) > 0:
                for lanelet in mapped_lanelets:
                    self._add_light_to_lanelet(lanelet.lanelet_id)
            else:
                rospy.logerr("CommonRoadMapProvider: TrafficLight ID - " +
                             str(light.mark_id) + " - did not match to any lanelet")

    def _visualize_scenario(self, sce: Scenario, prob: PlanningProblem = None):
        plt.figure(figsize=(50, 50))
        draw_object(sce, draw_params={'time_begin': 0})
        if prob is not None:
            draw_object(prob)
        plt.gca().set_aspect('equal')
        plt.savefig("route_.png")
        plt.close()

    def _add_light_to_lanelet(self, lanelet_id: int):
        self.cur_light_id += 1
        pos = self.map_cr.lanelet_network.find_lanelet_by_id(lanelet_id).center_vertices[-1]
        traffic_light = TrafficLight(self.cur_light_id, [], pos)
        id_set = set()
        id_set.add(lanelet_id)
        self.map_cr.lanelet_network.add_traffic_light(traffic_light, lanelet_ids=deepcopy(id_set))

    def _get_lanelet_orientation_to_light(self, lanelet: Lanelet, use_end: bool = True):
        index_1 = 0
        index_2 = 1
        if use_end:
            index_1 = -2
            index_2 = -1
        prev_pos = lanelet.center_vertices[index_1]
        pos = lanelet.center_vertices[index_2]

        # describes the relativ position of the pos to the prev pos
        rel_x = 1 if (pos[0] - prev_pos[0]) >= 0 else -1
        rel_y = 1 if (prev_pos[1] - pos[1]) >= 0 else -1

        euler_angle_yaw = math.degrees(math.atan2(rel_y * abs(pos[1] - prev_pos[1]), rel_x * abs(pos[0] - prev_pos[0])))

        # get orientation without multiple rotations
        euler_angle_yaw = euler_angle_yaw % 360
        # get the positive angle
        if euler_angle_yaw < 0:
            euler_angle_yaw = euler_angle_yaw + 360

        return euler_angle_yaw

    def _generate_dummy_planning_problem(self) -> PlanningProblem:
        """
        Generate the planning problem by setting the starting point and generating a target region
        :return: dummy generated problem
        """

        start_state: State = State(position=np.array([0, 0]), velocity=0,
                                   time_step=0, slip_angle=0, yaw_rate=0, orientation=0)

        circle = Circle(1, center=np.array([0, 0]))
        goal_state: State = State(position=circle, time_step=Interval(0, 10000.0))

        goal_region: GoalRegion = GoalRegion([goal_state], None)

        # return planning problem with start_state and goal_region
        return PlanningProblem(0, start_state, goal_region)


def main():
    CommonRoadMapProvider(True)
    rospy.spin()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
