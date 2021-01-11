from psaf_messages.msg import Obstacle
from psaf_planning.global_planner.path_provider_abstract import PathProviderAbstract
from psaf_planning.global_planner.path_provider_common_roads import PathProviderCommonRoads
import rospy
from lanelet2.core import GPSPoint
from geometry_msgs.msg import Point
from std_msgs.msg import String
import numpy as np
from commonroad.scenario.lanelet import Lanelet, LineMarking
# import necessary classes from different modules
from commonroad.geometry.shape import Rectangle
from commonroad.scenario.obstacle import StaticObstacle, ObstacleType
from commonroad.scenario.trajectory import State


class PathSupervisorCommonRoads(PathProviderCommonRoads):
    def __init__(self, init_rospy: bool = False, enable_debug: bool = False):
        rospy.init_node('TEST2', anonymous=True)
        super(PathSupervisorCommonRoads, self).__init__(init_rospy=not init_rospy, enable_debug=enable_debug)
        self.busy: bool = False
        self.intersections = self._map_intersection_and_lanelets()
        rospy.Subscriber("/psaf/planning/obstacle", Obstacle, self._callback_obstacle)
        self.obstacles = {}

    def _callback_obstacle(self, data: Obstacle):
        rospy.logerr("PathSupervisor: TEst")
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
            rospy.logerr("HJI")
            cur_lanelet = self.map.lanelet_network.find_lanelet_by_id(matching_lanelet[0][0])
            merged_lanelet: Lanelet = cur_lanelet
            # collect all successor lanelet that lead to an intersection and create one long one
            rospy.logerr("HJI2")
            matching_lanelet: Lanelet = self.map.lanelet_network.find_lanelet_by_id(matching_lanelet[0][0])
            next_lane = self.map.lanelet_network.find_lanelet_by_id(matching_lanelet._successor[0])
            index = 0
            for i in range(0, len(self.map.lanelet_network.lanelets)):
                if self.map.lanelet_network.lanelets[i].lanelet_id == next_lane.lanelet_id:
                    index = i
            o = self.map.lanelet_network.lanelets[index]
            o._predecessor = []
            prev_lane = self.map.lanelet_network.find_lanelet_by_id(matching_lanelet._predecessor[0])
            for i in range(0, len(self.map.lanelet_network.lanelets)):
                if self.map.lanelet_network.lanelets[i].lanelet_id == prev_lane.lanelet_id:
                    index = i
            o = self.map.lanelet_network.lanelets[index]
            o._successor = []
            self.map.lanelet_network.cleanup_lanelet_references()
            for i in range(0, 5):
            #while cur_lanelet.lanelet_id not in self.intersections:
                if len(cur_lanelet.successor) == 0:
                    break
                cur_lanelet = self.map.lanelet_network.find_lanelet_by_id(cur_lanelet.successor[0])
                merged_lanelet = Lanelet.merge_lanelets(merged_lanelet, cur_lanelet)
            static_obstacle_id = self.map.generate_object_id()
            rospy.logerr("HJI3")
            static_obstacle_type = ObstacleType.PARKED_VEHICLE
            #static_obstacle_shape = merged_lanelet.convert_to_polygon()
            static_obstacle_shape = Rectangle(width = 4.5, length = 4.5)
            orientation = self.vehicle_status.get_status().get_orientation_as_euler()[2]
            rospy.logerr("PathSupervisor: x{} y{} !!".format(obs_pos_x, obs_pos_y))
            static_obstacle_initial_state = State(position=np.array([obs_pos_x, obs_pos_y]),
                                                  orientation=orientation, time_step=0)

            # feed in the required components to construct a static obstacle
            static_obstacle = StaticObstacle(static_obstacle_id, static_obstacle_type, static_obstacle_shape,
                                             static_obstacle_initial_state)

            # add the static obstacle to the scenario
            self.map.add_objects(static_obstacle)
            self._replan()
        return True

    def _remove_obstacle(self, obstacle: Obstacle):
        rospy.loginfo("PathSupervisor: Remove obstacle")
        curr_pos_gps: Point = self._get_current_position()
        if obstacle.id not in self.obstacles:
            rospy.logerr("PathSupervisor: replanning aborted, obstacle doesn't exists !!")
            self.status_pub.publish("Replanning aborted, obstacle doesn't exists")
            return False

    def _get_current_position(self) -> Point:
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
