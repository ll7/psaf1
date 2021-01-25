#!/usr/bin/env python

import sys

from psaf_messages.msg import Obstacle
from psaf_planning.global_planner.path_provider_abstract import PathProviderAbstract
from psaf_planning.global_planner.path_provider_common_roads import PathProviderCommonRoads
import rospy
from lanelet2.core import GPSPoint
from geometry_msgs.msg import Point
import numpy as np
from commonroad.scenario.lanelet import Lanelet
from commonroad.geometry.shape import Rectangle
from commonroad.scenario.obstacle import StaticObstacle, ObstacleType
from commonroad.scenario.trajectory import State
from copy import deepcopy


class PathSupervisorCommonRoads(PathProviderCommonRoads):
    def __init__(self, init_rospy: bool = False, enable_debug: bool = False):
        if init_rospy:
            rospy.init_node('PathSupervisorCommonRoads', anonymous=True)
        super(PathSupervisorCommonRoads, self).__init__(init_rospy=not init_rospy, enable_debug=enable_debug)
        self.busy: bool = False
        rospy.Subscriber("/psaf/planning/obstacle", Obstacle, self._callback_obstacle)
        self.obstacles = {}
        self.last_id = -1
        self.status_pub.publish("Init Done")

    def _callback_obstacle(self, obstacle: Obstacle):
        if not self.busy and self.manager.map is not None and len(self.path.poses) > 0:
            self.busy = True
            # check if an old
            #if self.last_id >= obstacle.id:
            #    rospy.logerr("PathSupervisor: replanning aborted, received an old replaning msg !!")
            #    self.status_pub.publish("Replanning aborted, received an old replaning msg")
            #    self.busy = False
            #    return
            self.last_id = obstacle.id
            self.status_pub.publish("Start Replanning")
            # create a clean slate
            self.manager.map = deepcopy(self.manager.original_map)
            self.manager.neighbourhood = deepcopy(self.manager.original_neighbourhood)
            self.manager.message_by_lanelet = deepcopy(self.manager.original_message_by_lanelet)
            self.manager.time_by_lanelet = deepcopy(self.manager.original_time_by_lanelet)
            for point in obstacle.obstacles:
                if self._add_obstacle(point):
                    self.status_pub.publish("Replanning done")
            # generate new plan
            self._replan()
            self.busy = False
        else:
            rospy.logerr("PathSupervisor: replanning aborted, contact support !!")
            self.status_pub.publish("Replanning aborted, contact support")

    def _add_obstacle(self, obstacle: Point):
        """
        Add obstacle to scenario
        """
        rospy.loginfo("PathSupervisor: Add obstacle")
        curr_pos: Point = self._get_current_position()
        obs_pos_x = obstacle.x
        obs_pos_y = obstacle.y
        car_lanelet = self.manager.map.lanelet_network.find_lanelet_by_position([np.array([curr_pos.x, curr_pos.y])])
        matching_lanelet = self.manager.map.lanelet_network.find_lanelet_by_position([np.array([obs_pos_x, obs_pos_y])])

        rospy.loginfo("found car in lanelet: " + str(matching_lanelet[0]))

        # check if the obstacle is within a lanelet
        if len(matching_lanelet[0]) == 0:
            rospy.logerr("PathSupervisor: Ignoring obstacle, obstacle not in a lanelet -> no interfering !!")
            self.status_pub.publish("Ignoring obstacle, obstacle not in a lanelet -> no interfering")
            return False
        if self.manager.message_by_lanelet[matching_lanelet[0][0]].isAtIntersection:
            rospy.logerr("PathSupervisor: Ignoring obstacle, obstacle on a intersection !!")
            self.status_pub.publish("Ignoring obstacle, obstacle on a intersection")
            return False
        lanelet: Lanelet = self.manager.map.lanelet_network.find_lanelet_by_id(car_lanelet[0][0])
        # case single road in current direction
        if lanelet.adj_left_same_direction is False and lanelet.adj_right_same_direction is False:
            rospy.logerr("PathSupervisor: Ignoring obstacle, single road in current direction !!")
            self.status_pub.publish("Ignoring obstacle, single road in current direction")
            return False
        # case at least one neighbouring lane and no solid line
        elif lanelet.adj_right_same_direction is not None or lanelet.adj_left_same_direction is not None:
            static_obstacle_id = self.manager.map.generate_object_id()
            static_obstacle_type = ObstacleType.PARKED_VEHICLE
            static_obstacle_shape = Rectangle(width=2, length=4.5)
            orientation = self.vehicle_status.get_status().get_orientation_as_euler()[2]
            static_obstacle_initial_state = State(position=np.array([obs_pos_x, obs_pos_y]),
                                                  orientation=orientation, time_step=0)

            # feed in the required components to construct a static obstacle
            static_obstacle = StaticObstacle(static_obstacle_id, static_obstacle_type, static_obstacle_shape,
                                             static_obstacle_initial_state)

            if car_lanelet[0][0] == matching_lanelet[0][0]:
                # add the static obstacle to the scenario
                split_ids = self.manager.update_network(matching_lanelet[0][0], Point(obs_pos_x, obs_pos_y, 0), curr_pos,
                                                      static_obstacle)
                split_point = Point(self.manager.map.lanelet_network.find_lanelet_by_id(split_ids[1]).center_vertices[0][0],
                                    self.manager.map.lanelet_network.find_lanelet_by_id(split_ids[1]).center_vertices[0][1], 0)
                self.manager.update_network(split_ids[0], curr_pos, split_point, None)
            else:
                if static_obstacle is not None:
                    self.manager.map.lanelet_network.find_lanelet_by_id(matching_lanelet[0][0]).add_static_obstacle_to_lanelet(
                        static_obstacle.obstacle_id)
                    self.manager.map.add_objects(static_obstacle)

        return True

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
        self.get_path_from_a_to_b(debug=self.enable_debug)
        self._trigger_move_base(self.path.poses[-1])
        rospy.loginfo("PathSupervisor: global planner plugin triggered")
        self.status_pub.publish("Replanning done")


def main():
    provider: PathProviderAbstract = PathSupervisorCommonRoads(init_rospy=True, enable_debug=False)
    rospy.spin()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
