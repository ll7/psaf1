#!/usr/bin/env python

from psaf_messages.msg import Obstacle
from psaf_planning.global_planner.path_provider_common_roads import PathProviderCommonRoads
import rospy
from lanelet2.core import GPSPoint
from geometry_msgs.msg import Point
import numpy as np
from commonroad.geometry.shape import Rectangle
from commonroad.scenario.obstacle import StaticObstacle, ObstacleType
from commonroad.scenario.trajectory import State
from copy import deepcopy
from typing import List
from commonroad.scenario.lanelet import Lanelet


class PathSupervisorCommonRoads(PathProviderCommonRoads):
    def __init__(self, init_rospy: bool = False, enable_debug: bool = False, respect_traffic_rules: bool = False):
        if init_rospy:
            rospy.init_node('PathSupervisorCommonRoads', anonymous=True)
        super(PathSupervisorCommonRoads, self).__init__(init_rospy=not init_rospy, enable_debug=enable_debug, respect_traffic_rules=respect_traffic_rules)
        self.busy: bool = False
        rospy.Subscriber("/psaf/planning/obstacle", Obstacle, self._callback_obstacle, queue_size=1)
        self.obstacles = {}
        self.last_id = -1
        self.status_pub.publish("Init Done")

    def _callback_obstacle(self, obstacle: Obstacle):
        if not self.busy and self.manager.map is not None and self.path_message.id > 0:
            self.busy = True
            # check if an old
            if self.last_id >= obstacle.id:
                rospy.logerr("PathSupervisor: replanning aborted, received an old replaning msg !!")
                self.status_pub.publish("Replanning aborted, received an old replaning msg")
                self.busy = False
                return
            self.last_id = obstacle.id
            self.status_pub.publish("Start Replanning")
            # create a clean slate
            self.manager.map = deepcopy(self.manager.original_map)
            self.manager.neighbourhood = deepcopy(self.manager.original_neighbourhood)
            self.manager.message_by_lanelet = deepcopy(self.manager.original_message_by_lanelet)
            # determine car lanelet
            curr_pos: Point = self._get_current_position()
            car_lanelet = self.manager.map.lanelet_network.find_lanelet_by_position(
                [np.array([curr_pos.x, curr_pos.y])])
            # check if car is on an lanelet
            if len(car_lanelet[0]) == 0:
                rospy.logerr("PathSupervisor: Car is not on a lanelet, abort !!")
                self.status_pub.publish("Car is not on a lanelet, abort !! ")
            else:
                # determine relevant lanelets for an obstacle
                car_lanelet = car_lanelet[0][0]
                relevant_lanelets = self._determine_relevant_lanelets(car_lanelet)
                real_obstacles = {}
                for obs in obstacle.obstacles:
                    matching_lanelet = self._get_obstacle_lanelet(relevant_lanelets, obs)
                    if matching_lanelet == -1:
                        rospy.logerr(
                            "PathSupervisor: Ignoring obstacle, obstacle not in a relevant lanelet -> no interfering !!")
                        self.status_pub.publish(
                            "Ignoring obstacle, obstacle not in a relevant lanelet -> no interfering")
                    else:
                        real_obstacles[matching_lanelet] = obs
                for lane_id in real_obstacles:
                    rospy.loginfo("PathSupervisor: Processing obstacle: {}".format(real_obstacles[lane_id]))
                    success, car_lanelet = self._add_obstacle(real_obstacles[lane_id], car_lanelet, curr_pos, relevant_lanelets)
                    relevant_lanelets = self._determine_relevant_lanelets(car_lanelet)
                # generate new plan
                self._replan()
            self.busy = False
        elif self.busy:
            rospy.logerr("PathSupervisor: Replanning aborted, still busy !!")
            self.status_pub.publish("Replanning aborted, still busy")
        elif self.path_message.id == 0:
            rospy.logerr("PathSupervisor: Replanning aborted, initial plan not valid !!")
            self.status_pub.publish("Replanning aborted, initial plan not valid")
        else:
            rospy.logerr("PathSupervisor: Replanning aborted, contact support !!")
            self.status_pub.publish("Replanning aborted, contact support")
        self.status_pub.publish("Replanning done")

    def _determine_relevant_lanelets(self, car_lanelet: int) -> List[int]:
        """
        Generates a list of all possible lanelets for an obstacle
        :param car_lanelet: id of the lanelet where the car currently located on
        :return: list of lanelets that are possible for obstacle insertion
        """
        relevant = set()
        lanelet: Lanelet = self.manager.map.lanelet_network.find_lanelet_by_id(car_lanelet)
        if not self.manager.message_by_lanelet[car_lanelet].isAtIntersection:
            relevant.add(car_lanelet)
        # add all neighbours and their successors, that are heading in the same direction
        # go through all right neighbours
        neighbour_lanelet = lanelet
        while True:
            tmp = neighbour_lanelet
            while neighbour_lanelet.adj_right_same_direction and neighbour_lanelet.adj_right is not None:
                if not self.manager.message_by_lanelet[neighbour_lanelet.lanelet_id].isAtIntersection:
                    relevant.add(neighbour_lanelet.lanelet_id)
                neighbour_lanelet = self.manager.map.lanelet_network.find_lanelet_by_id(neighbour_lanelet.adj_right)
            # go through all left neighbours
            neighbour_lanelet = tmp
            while neighbour_lanelet.adj_left_same_direction and neighbour_lanelet.adj_left is not None:
                if not self.manager.message_by_lanelet[neighbour_lanelet.lanelet_id].isAtIntersection:
                    relevant.add(neighbour_lanelet.lanelet_id)
                neighbour_lanelet = self.manager.map.lanelet_network.find_lanelet_by_id(neighbour_lanelet.adj_left)
            if len(neighbour_lanelet.successor) > 1:
                break
            neighbour_lanelet = self.manager.map.lanelet_network.find_lanelet_by_id(neighbour_lanelet.successor[0])

        return list(relevant)

    def _get_obstacle_lanelet(self, relevant: List[int], obstacle: Point) -> int:
        """
        Generates a list of all possible lanelets for an obstacle
        :param relevant: list of lanelets that are possible for obstacle insertion
        :param obstacle: id of the lanelet where the car currently located on
        :return: id of the obstacle lanelet, -1 if the didn't match a lanelet in the relevant list
        """
        matching: int = -1
        for lane in relevant:
            lanelet = self.manager.map.lanelet_network.find_lanelet_by_id(lane)
            point_list = np.array([[obstacle.x, obstacle.y], [0, 0]])
            if lanelet.contains_points(point_list)[0]:
                matching = lane
            break
        return matching

    def _add_obstacle(self, obstacle: Point, car_lanelet: int, curr_pos: Point, relevant: List[int]):
        """
        Add obstacle to scenario
        :param obstacle: Point of the obstacle (x, y)
        :param car_lanelet: id of the lanelet where the car currently located on
        :param curr_pos: current position of the car (x, y)
        :param relevant: list of lanelets that are possible for obstacle insertion
        """
        rospy.loginfo("PathSupervisor: Add obstacle")
        obs_pos_x = obstacle.x
        obs_pos_y = obstacle.y
        matching_lanelet = self._get_obstacle_lanelet(relevant, obstacle)
        # check if the obstacle is within a lanelet
        if matching_lanelet == -1:
            rospy.logerr("PathSupervisor: Ignoring obstacle, obstacle not in a relevant lanelet -> no interfering !!")
            self.status_pub.publish("Ignoring obstacle, obstacle not in a relevant lanelet -> no interfering")
            return False, car_lanelet

        static_obstacle_id = self.manager.map.generate_object_id()
        static_obstacle_type = ObstacleType.PARKED_VEHICLE
        static_obstacle_shape = Rectangle(width=2, length=4.5)
        orientation = self.vehicle_status.get_status().get_orientation_as_euler()[2]
        static_obstacle_initial_state = State(position=np.array([obs_pos_x, obs_pos_y]),
                                              orientation=orientation, time_step=0)

        # feed in the required components to construct a static obstacle
        static_obstacle = StaticObstacle(static_obstacle_id, static_obstacle_type, static_obstacle_shape,
                                         static_obstacle_initial_state)

        if car_lanelet == matching_lanelet:
            # add the static obstacle to the scenario
            split_ids = self.manager.update_network(matching_lanelet, Point(obs_pos_x, obs_pos_y, 0),
                                                    curr_pos,
                                                    static_obstacle)
            if split_ids[0] is not None:
                car_lanelet = split_ids[0]
                split_point = Point(
                    self.manager.map.lanelet_network.find_lanelet_by_id(split_ids[1]).center_vertices[0][0],
                    self.manager.map.lanelet_network.find_lanelet_by_id(split_ids[1]).center_vertices[0][1], 0)
                split_ids = self.manager.update_network(split_ids[0], curr_pos, split_point, None)
                if split_ids[0] is not None:
                    car_lanelet = split_ids[0]
        else:
            if static_obstacle is not None:
                self.manager.map.lanelet_network.find_lanelet_by_id(
                    matching_lanelet).add_static_obstacle_to_lanelet(
                    static_obstacle.obstacle_id)
                self.manager.map.add_objects(static_obstacle)

        return True, car_lanelet

    def _get_current_position(self) -> Point:
        """
        Returns the current x, y and z position of the car
        :return: Point of the car
        """
        curr_pos_gps = self.GPS_Sensor.get_position()
        gps_point = GPSPoint(curr_pos_gps.latitude, curr_pos_gps.longitude, curr_pos_gps.altitude)
        position_3d = self.projector.forward(gps_point)
        return Point(position_3d.x, position_3d.y, position_3d.z)

    def _replan(self):
        """
        Replan after change in observed obstacles
        """
        # update current position
        self.start = self.GPS_Sensor.get_position()
        # and orientation
        self.start_orientation = self.vehicle_status.get_status().get_orientation_as_euler()
        rospy.loginfo("PathSupervisor: Replanning instruction received")
        self.get_path_from_a_to_b()
        self.status_pub.publish("Replanning done")

    def _reset_map(self):
        # reset obstacle msg id
        self.last_id = -1
        # first reset map
        if self.manager is not None:
            # create clean slate
            self.manager.map = deepcopy(self.manager.original_map)
            self.manager.neighbourhood = deepcopy(self.manager.original_neighbourhood)
            self.manager.message_by_lanelet = deepcopy(self.manager.original_message_by_lanelet)


def main():
    respect_traffic_rules = rospy.get_param('/path_provider/respect_traffic_rules', False)
    provider = PathSupervisorCommonRoads(init_rospy=False, enable_debug=True, respect_traffic_rules=bool(respect_traffic_rules))
    rospy.spin()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
