from psaf_messages.msg import Obstacle
from psaf_planning.global_planner.path_provider_abstract import PathProviderAbstract
from psaf_planning.global_planner.path_provider_common_roads import PathProviderCommonRoads
import rospy
from lanelet2.core import GPSPoint
from geometry_msgs.msg import Point
from std_msgs.msg import String
import numpy as np
from commonroad.scenario.lanelet import Lanelet, LineMarking
# import necesary classes from different modules
from commonroad.geometry.shape import Rectangle
from commonroad.scenario.obstacle import StaticObstacle, ObstacleType
from commonroad.scenario.trajectory import State


class PathSupervisorCommonRoads(PathProviderCommonRoads):
    def __init__(self, init_rospy: bool = False, enable_debug: bool = False):
        super(PathSupervisorCommonRoads, self).__init__(init_rospy=not init_rospy, enable_debug=enable_debug)
        if init_rospy:
            # initialize node
            rospy.init_node('PathSupervisor', anonymous=True)
        self.busy: bool = False
        rospy.Subscriber("/psaf/planning/obstacle", Obstacle, self._callback_obstacle)
        self.status_pub = rospy.Publisher('/psaf/status', String, queue_size=10)
        self.obstacles = {}

    def _callback_obstacle(self, data: Obstacle):
        if not self.busy and self.map is not None:
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
        lanelet: Lanelet = car_lanelet[0][0]
        # case single road in current direction
        if lanelet.adj_left_same_direction is False or lanelet.adj_right_same_direction is False:
            rospy.logerr("PathSupervisor: Replanning aborted, single road in current direction !!")
            self.status_pub.publish("Replanning aborted, single road in current direction")
            return False
        # case at least one neighbouring lane and no solid line
        elif lanelet.line_marking_left_vertices is not None and \
                lanelet.line_marking_left_vertices is not LineMarking.SOLID or \
                lanelet.line_marking_right_vertices is not None and \
                lanelet.line_marking_right_vertices is not LineMarking.SOLID:
            # generate the static obstacle according to the specification, refer to API for details of input parameters
            static_obstacle_id = self.map.generate_object_id()
            static_obstacle_type = ObstacleType.PARKED_VEHICLE
            static_obstacle_shape = Rectangle(width=2.0, length=4.5)
            orientation = self.vehicle_status.get_status().get_orientation_as_euler()[2]
            static_obstacle_initial_state = State(position=np.array([obs_pos_x, obs_pos_y]), orientation=orientation, time_step=0)

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


def main():
    provider: PathProviderAbstract = PathSupervisorCommonRoads(init_rospy=True, enable_debug=False)
    rospy.spin()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
