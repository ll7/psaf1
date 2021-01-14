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


class PathSupervisorCommonRoads(PathProviderCommonRoads):
    def __init__(self, init_rospy: bool = False, enable_debug: bool = False):
        if init_rospy:
            rospy.init_node('PathSupervisorCommonRoads', anonymous=True)
        super(PathSupervisorCommonRoads, self).__init__(init_rospy=not init_rospy, enable_debug=enable_debug)
        self.busy: bool = False
        self.intersections = self._map_intersection_and_lanelets()
        rospy.Subscriber("/psaf/planning/obstacle", Obstacle, self._callback_obstacle)
        self.obstacles = {}

    def _callback_obstacle(self, data: Obstacle):
        if not self.busy and self.map is not None and len(self.path.poses) > 0:
            self.busy = True
            if data.action is data.ACTION_ADD:
                self.status_pub.publish("Start Replanning")
                if self._add_obstacle(data):
                    self.status_pub.publish("Replanning done")
            elif data.action is data.ACTION_REMOVE:
                self.status_pub.publish("Start Replanning")
                if self._remove_obstacle(data):
                    self.status_pub.publish("Replanning done")
            else:
                rospy.logerr("PathSupervisor: replanning aborted, unknown action !!")
                self.status_pub.publish("Replanning aborted, unknown action")
            self.busy = False
        else:
            rospy.logerr("PathSupervisor: replanning aborted, unknown action !!")
            self.status_pub.publish("Replanning aborted, unknown action")

    def _add_obstacle(self, obstacle: Obstacle):
        """
        Add obstacle to scenario
        """
        rospy.loginfo("PathSupervisor: Add obstacle")
        curr_pos: Point = self._get_current_position()
        # check if the obstacle already exists
        if obstacle.id in self.obstacles:
            rospy.logerr("PathSupervisor: Replanning aborted, obstacle already exists !!")
            self.status_pub.publish("Replanning aborted, obstacle already exists")
            return False
        obs_pos_x = curr_pos.x + obstacle.x
        obs_pos_y = curr_pos.y + obstacle.y
        car_lanelet = self.map.lanelet_network.find_lanelet_by_position([np.array([curr_pos.x, curr_pos.y])])
        matching_lanelet = self.map.lanelet_network.find_lanelet_by_position([np.array([obs_pos_x, obs_pos_y])])
        # check if the obstacle is within a lanelet
        if len(matching_lanelet[0]) == 0:
            rospy.logerr("PathSupervisor: Replanning aborted, obstacle not in a lanelet -> no interfering !!")
            self.status_pub.publish("Replanning aborted, obstacle not in a lanelet -> no interfering")
            return False
        lanelet: Lanelet = self.map.lanelet_network.find_lanelet_by_id(car_lanelet[0][0])
        # case single road in current direction
        if lanelet.adj_left_same_direction is False or lanelet.adj_right_same_direction is False:
            rospy.logerr("PathSupervisor: Replanning aborted, single road in current direction !!")
            self.status_pub.publish("Replanning aborted, single road in current direction")
            return False
        # case at least one neighbouring lane and no solid line
        elif lanelet.adj_right_same_direction is not None or lanelet.adj_left_same_direction is not None:
            static_obstacle_id = self.map.generate_object_id()
            static_obstacle_type = ObstacleType.PARKED_VEHICLE
            static_obstacle_shape = Rectangle(width=2, length=4.5)
            orientation = self.vehicle_status.get_status().get_orientation_as_euler()[2]
            static_obstacle_initial_state = State(position=np.array([obs_pos_x, obs_pos_y]),
                                                  orientation=orientation, time_step=0)

            # feed in the required components to construct a static obstacle
            static_obstacle = StaticObstacle(static_obstacle_id, static_obstacle_type, static_obstacle_shape,
                                             static_obstacle_initial_state)

            # add the static obstacle to the scenario
            front_split_id = self._update_network(matching_lanelet[0][0], Point(obs_pos_x, obs_pos_y, 0), static_obstacle)
            self._update_network(front_split_id[0], curr_pos, None)
            self.map.lanelet_network.cleanup_lanelet_references()
            # add obstacle to dict
            self.obstacles[obstacle.id] = static_obstacle
            self._replan()
        return True

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

    def _update_network(self, matching_lanelet_id: int, modify_point: Point, static_obstacle):
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
        if self.map.lanelet_network.find_lanelet_by_id(matching_lanelet_id).adj_left is None:
            right_1, right_2 = self._modify_lanelet(
                self.map.lanelet_network.find_lanelet_by_id(matching_lanelet_id).adj_right,
                modify_point)
        if self.map.lanelet_network.find_lanelet_by_id(matching_lanelet_id).adj_right is None:
            left_1, left_2 = self._modify_lanelet(
                self.map.lanelet_network.find_lanelet_by_id(matching_lanelet_id).adj_left,
                modify_point)
        # split obstacle lanelet
        matching_1, matching_2 = self._modify_lanelet(matching_lanelet_id, modify_point)
        # update neighbourhood
        if right_1 is not None:
            self.map.lanelet_network.find_lanelet_by_id(right_1)._adj_left = matching_1
            self.map.lanelet_network.find_lanelet_by_id(right_2)._adj_left = matching_2
            self.map.lanelet_network.find_lanelet_by_id(matching_1)._adj_right = right_1
            self.map.lanelet_network.find_lanelet_by_id(matching_2)._adj_right = right_2
        if left_1 is not None:
            self.map.lanelet_network.find_lanelet_by_id(left_1)._adj_right = matching_1
            self.map.lanelet_network.find_lanelet_by_id(left_2)._adj_right = matching_2
            self.map.lanelet_network.find_lanelet_by_id(matching_1)._adj_left = left_1
            self.map.lanelet_network.find_lanelet_by_id(matching_2)._adj_left = left_2
        # add obstacle
        if static_obstacle is not None:
            self.map.lanelet_network.find_lanelet_by_id(matching_2).add_static_obstacle_to_lanelet(
                static_obstacle.obstacle_id)
            self.map.add_objects(static_obstacle)
        return matching_1, matching_2

    def _modify_lanelet(self, lanelet_id: int, modify_point: Point):
        """Splits a lanelet ata certain point

        :param lanelet_id: lanelet to be split
        :param modify_point: point of split
        :return: ids of the split lanelet
        """
        # create new ids
        id_lane_1 = self._generate_lanelet_id(id_start=lanelet_id)
        id_lane_2 = self._generate_lanelet_id(id_start=lanelet_id, exclude=id_lane_1)
        # make a local copy of the lanelet to be removed
        lanelet_copy = self.map.lanelet_network.find_lanelet_by_id(lanelet_id)
        # delete lanelet
        del self.map.lanelet_network._lanelets[lanelet_id]
        # bounds lanelet1
        lanelet_center_list = lanelet_copy.center_vertices.tolist()
        sep_index = lanelet_center_list.index(min(lanelet_center_list, key=lambda
            pos: self._euclidean_2d_distance_from_to_position(pos, modify_point, use_posestamped=False)))
        if sep_index > 1:
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
            # then add "back" to the lanelet_network
            self.map.lanelet_network.add_lanelet(lanelet_1)
            self.map.lanelet_network.add_lanelet(lanelet_2)

            return id_lane_1, id_lane_2

    def _remove_obstacle(self, obstacle: Obstacle):
        """
        Removes an obstacle from the scenario
        """
        rospy.loginfo("PathSupervisor: Remove obstacle")
        curr_pos_gps: Point = self._get_current_position()
        if obstacle.id not in self.obstacles:
            rospy.logerr("PathSupervisor: replanning aborted, obstacle doesn't exists !!")
            self.status_pub.publish("Replanning aborted, obstacle doesn't exists")
            return False
        self.map.remove_obstacle(obstacle.id)

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

    def _map_intersection_and_lanelets(self):
        """
        Function to map the id of a lanelet to the associated intersection id
        :return: dict with [str: "#lanelet id"] -> int: intersection id
        """
        lanelet_intersection_map = {}
        for intersection in self.map.lanelet_network.intersections:
            for incoming in intersection.incomings:
                for lanelet in incoming.incoming_lanelets:
                    lanelet_intersection_map[str(lanelet)] = intersection.intersection_id

        return lanelet_intersection_map


def main():
    provider: PathProviderAbstract = PathSupervisorCommonRoads(init_rospy=True, enable_debug=True)
    rospy.spin()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
