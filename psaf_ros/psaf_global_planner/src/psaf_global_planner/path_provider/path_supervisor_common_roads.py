#!/usr/bin/env python

from psaf_messages.msg import Obstacle
from psaf_global_planner.path_provider.path_provider_common_roads import PathProviderCommonRoads
import rospy
from lanelet2.core import GPSPoint
from geometry_msgs.msg import Point, Pose
import numpy as np
from commonroad.geometry.shape import Rectangle
from commonroad.scenario.obstacle import StaticObstacle, ObstacleType
from commonroad.scenario.trajectory import State
from copy import deepcopy
from typing import List
from commonroad.scenario.lanelet import Lanelet


class PathSupervisorCommonRoads(PathProviderCommonRoads):
    def __init__(self, init_rospy: bool = False,
                 enable_debug: bool = False,
                 respect_traffic_rules: bool = False,
                 export_path: bool = False):

        if init_rospy:
            rospy.init_node('PathSupervisorCommonRoads', anonymous=True)
        super(PathSupervisorCommonRoads, self).__init__(init_rospy=not init_rospy, enable_debug=enable_debug,
                                                        respect_traffic_rules=respect_traffic_rules,
                                                        export_path=export_path)
        self.busy: bool = False
        rospy.Subscriber("/psaf/planning/obstacle", Obstacle, self._callback_obstacle, queue_size=1)
        self.obstacles = {}
        self.last_id = -1
        self.status_pub.publish("Init Done")

    def _callback_obstacle(self, obstacle: Obstacle):
        """
        Callback function of psaf planning obstacle subscriber
        :param obstacle: the received obstacle data
        """
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
            rospy.loginfo("PathSupervisor: Start Replanning")
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
                self.busy = False
                return
            else:
                # determine relevant lanelets for an obstacle
                car_lanelet = car_lanelet[0][0]
                relevant_lanelets = self._determine_relevant_lanelets(car_lanelet)
                self._log_debug("--------------------")
                self._log_debug("Car lanelet: {}".format(car_lanelet))
                self._log_debug("--------------------")
                self._log_debug("Relevant:")
                for lane in relevant_lanelets:
                    self._log_debug("\t {}".format(lane))
                real_obstacles = {}
                self._log_debug("--------------------")
                self._log_debug("Trying to match {} obstacles".format(len(obstacle.obstacles)))
                self._log_debug("Matched:")
                for obs in obstacle.obstacles:
                    matching_lanelet = self._get_obstacle_lanelet(relevant_lanelets, obs)
                    if self.enable_debug:
                        obs_lane = self.manager.map.lanelet_network.find_lanelet_by_position([np.array([obs.x, obs.y])])
                        if obs_lane:
                            self._log_debug("\t (Matched without relevant: {})".format(matching_lanelet))
                    if matching_lanelet == -1:
                        self._log_debug(
                            "PathSupervisor: Ignoring obstacle, obstacle not in a relevant lanelet -> no interfering !!")
                        if self.enable_debug:
                            self.status_pub.publish(
                                "Ignoring obstacle, obstacle not in a relevant lanelet -> no interfering")
                    else:
                        self._log_debug("\t {}".format(matching_lanelet))
                        real_obstacles[matching_lanelet] = obs
                self._log_debug("--------------------")
                for lane_id in real_obstacles:
                    rospy.loginfo("PathSupervisor: Processing obstacle: {}".format(real_obstacles[lane_id]))
                    success, car_lanelet = self._add_obstacle(real_obstacles[lane_id], car_lanelet, curr_pos,
                                                              relevant_lanelets)
                    if not success:
                        # if a lanelet network error occurred, abort mission
                        rospy.logerr("PathSupervisor: Replanning aborted, split failed !!")
                        self.status_pub.publish("Replanning aborted, split failed")
                        self.busy = False
                        return
                    relevant_lanelets = self._determine_relevant_lanelets(car_lanelet)
                # generate new plan
                if len(real_obstacles) != 0:
                    self._replan()
                    self.status_pub.publish("PathSupervisor: finished")
                    rospy.loginfo("PathSupervisor: finished!!")
                else:
                    rospy.logerr("PathSupervisor: Replanning aborted, no matched obstacle !!")
                    self.status_pub.publish("Replanning aborted, no matched obstacle")
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

    def _determine_relevant_lanelets(self, car_lanelet: int) -> List[int]:
        """
        Generates a list of all possible lanelets for an obstacle
        :param car_lanelet: id of the lanelet where the car currently located on
        :return: list of lanelets that are possible for obstacle insertion
        """
        relevant = set()
        lanelet: Lanelet = self.manager.map.lanelet_network.find_lanelet_by_id(car_lanelet)
        # add all neighbours and their successors, that are heading in the same direction
        # go through all right neighbours
        neighbour_lanelet = lanelet
        while True:
            if not self.manager.message_by_lanelet[neighbour_lanelet.lanelet_id].isAtIntersection:
                relevant.add(neighbour_lanelet.lanelet_id)
            tmp = neighbour_lanelet
            while neighbour_lanelet.adj_right_same_direction and neighbour_lanelet.adj_right is not None:
                neighbour_lanelet = self.manager.map.lanelet_network.find_lanelet_by_id(neighbour_lanelet.adj_right)
                if not self.manager.message_by_lanelet[neighbour_lanelet.lanelet_id].isAtIntersection:
                    relevant.add(neighbour_lanelet.lanelet_id)
            # go through all left neighbours
            neighbour_lanelet = tmp
            while neighbour_lanelet.adj_left_same_direction and neighbour_lanelet.adj_left is not None:
                neighbour_lanelet = self.manager.map.lanelet_network.find_lanelet_by_id(neighbour_lanelet.adj_left)
                if not self.manager.message_by_lanelet[neighbour_lanelet.lanelet_id].isAtIntersection:
                    relevant.add(neighbour_lanelet.lanelet_id)
            if len(neighbour_lanelet.successor) != 0 and \
                    self.manager.message_by_lanelet[neighbour_lanelet.successor[0]].isAtIntersection:
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
            poly = self.manager.map.lanelet_network.find_lanelet_by_id(lane).convert_to_polygon()
            if poly.contains_point(np.array([obstacle.x, obstacle.y])):
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
        :return: true if the insertion was successful additionally the lanelet_id of the lanelet the car currently resides on
        """
        rospy.loginfo("PathSupervisor: Add obstacle")
        success = False
        obs_pos_x = obstacle.x
        obs_pos_y = obstacle.y
        matching_lanelet = self._get_obstacle_lanelet(relevant, obstacle)
        # check if the obstacle is within a lanelet
        if matching_lanelet == -1:
            rospy.logerr("PathSupervisor: Ignoring obstacle, obstacle not in a relevant lanelet anymore -> no "
                         "interfering !!")
            self.status_pub.publish("Ignoring obstacle, obstacle not in a relevant lanelet anymore-> no interfering")
            return success, car_lanelet

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
                success = True
                # split_point = Point(
                #    self.manager.map.lanelet_network.find_lanelet_by_id(split_ids[1]).center_vertices[0][0],
                #    self.manager.map.lanelet_network.find_lanelet_by_id(split_ids[1]).center_vertices[0][1], 0)
                # split_ids = self.manager.update_network(split_ids[0], curr_pos, split_point, None)
                #if split_ids[0] is not None:
                #    car_lanelet = split_ids[0]
                #   success = True
                #else:
                #    self._log_debug("PathSupervisor: Double Split_ids[0] is None!")
            else:
                self._log_debug("PathSupervisor: First Split_ids[0] is None!")
        else:
            if static_obstacle is not None:
                self.manager.map.lanelet_network.find_lanelet_by_id(
                    matching_lanelet).add_static_obstacle_to_lanelet(
                    static_obstacle.obstacle_id)
                self.manager.map.add_objects(static_obstacle)
                success = True
            else:
                self._log_debug("PathSupervisor: Static obstacle is None!")

        return success, car_lanelet

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
        # update current position and orientation
        start = self.GPS_Sensor.get_position()
        start = self.projector.forward(GPSPoint(start.latitude, start.longitude, start.altitude))
        start_orientation = self.vehicle_status.get_status().orientation
        self.start = Pose(position=Point(x=start.x, y=start.y, z=start.z), orientation=start_orientation)
        rospy.loginfo("PathSupervisor: Replanning instruction received")
        self.get_path_from_a_to_b()
        self.status_pub.publish("Replanning done")

    def _reset_map(self):
        """
        Reset the current knowledge to the unchanged originals
        """
        # reset obstacle msg id
        self.last_id = -1
        # first reset map
        if self.manager is not None:
            # create clean slate
            self.manager.map = deepcopy(self.manager.original_map)
            self.manager.neighbourhood = deepcopy(self.manager.original_neighbourhood)
            self.manager.message_by_lanelet = deepcopy(self.manager.original_message_by_lanelet)

    def _log_debug(self, msg):
        """
        Log the given message if enable_debug is set to True
        :param msg: the message
        """
        if self.enable_debug:
            rospy.loginfo(msg)


def main():
    respect_traffic_rules = rospy.get_param('/path_provider/respect_traffic_rules', False)
    export_path = rospy.get_param('/path_provider/export_path', False)
    enable_debug = rospy.get_param('/path_provider/enable_debug', False)
    provider = PathSupervisorCommonRoads(init_rospy=False, enable_debug=bool(enable_debug),
                                         respect_traffic_rules=bool(respect_traffic_rules),
                                         export_path=bool(export_path))
    rospy.spin()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
