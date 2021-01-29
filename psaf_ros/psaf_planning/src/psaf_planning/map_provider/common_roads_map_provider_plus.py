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

from psaf_planning.map_provider.landmark_provider import LandMarkProvider
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
            rospy.init_node('commonRoadMapProvider', anonymous=True)
        rospy.loginfo("hi")
        super(CommonRoadMapProvider, self).__init__()
        self.intersection = {}
        self.map_cr: Scenario = None
        self.landmark_provider: LandMarkProvider = None
        self.cur_light_id = -1
        self.planning_problem = None

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
            rospy.loginfo("CommonRoadMapProvider: DONE DONE")
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

    def _detect_intersections(self):
        for lanelet in self.map_cr.lanelet_network.lanelets:
            # get the successor of the successor of the given lanelet
            lane = lanelet
            if len(lane.successor) == 0:
                return False
            succ = lanelet.successor[0]
            lane = self.map_cr.lanelet_network.find_lanelet_by_id(succ)

            # check for the amount of predecessor
            self.intersection[lanelet.lanelet_id] = len(lane.predecessor) > 1

    def _traffic_lights_to_scenario(self, lights):
        for light in lights:
            lanelet: Lanelet = self._find_nearest_lanlet(light)
            is_intersection = self.intersection[lanelet.lanelet_id]
            # check whether the orientation of the traffic light and the corresponding lanelet matches
            # allow a absolut tolerance of 20 degrees to consider a traffic light associated with the lanelet
            if lanelet.lanelet_id == 248:
                print("------------Falsch-------------")
                print("{} {} {} {}".format(light.x, light.y, light.orientation, light.mark_id))
                print("{}".format(self._get_lanelet_orientation_to_light(lanelet, is_intersection)))
                print("------------Falsch-------------")
                continue
            angle_diff = 180 - abs(
                abs(light.orientation - self._get_lanelet_orientation_to_light(lanelet, is_intersection)) - 180)
            # if the orientation matches
            if angle_diff < 20:
                # if the nearest lanelet is not in an intersection find the right one
                if not is_intersection:
                    # calc the orientation of all predecessors
                    for pred in lanelet.predecessor:
                        lanelet_under_obs = self.map_cr.lanelet_network.find_lanelet_by_id(pred)
                        is_intersection = self.intersection[pred]
                        if not is_intersection:
                            print("NONONONO broken, contact the support ")
                        else:
                            orientation_dif = 180 - abs(
                                abs(light.orientation - self._get_lanelet_orientation_to_light(lanelet_under_obs,
                                                                                               is_intersection)) - 180)
                            # if the lanelet with the right orientation is found, replace the nearest with this one
                            if orientation_dif < 20:
                                lanelet = deepcopy(lanelet_under_obs)
                                break

                # add traffic light to the predecessor of the lanelet
                obs_id = lanelet.predecessor[0]
                self._add_light_to_lanelet(obs_id)
                # add traffic light to all adj lanelets that are heading in the same direction
                while True:
                    if self.map_cr.lanelet_network.find_lanelet_by_id(obs_id).adj_left_same_direction and \
                            self.map_cr.lanelet_network.find_lanelet_by_id(obs_id).adj_left is not None:
                        obs_id = self.map_cr.lanelet_network.find_lanelet_by_id(obs_id).adj_left
                        self._add_light_to_lanelet(obs_id)
                    else:
                        # no more lanelets that are heading in the same direction
                        break
            else:
                print("TOBI WILL FIX IT")

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

    def _generate_dummy_planning_problem(self) ->PlanningProblem:
        """
        Generate the planning problem by setting the starting point and generating a target region
        :param start:  x,y,z coordinates of the current staring position
        :param target: x,y,z coordinates of the current goal position
        :return: status of the generated problem
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
