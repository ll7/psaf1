import math
import sys

import actionlib
import rosbag
import rospy
import tf
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
from mbf_msgs.msg import MoveBaseGoal
from move_base_msgs.msg import MoveBaseAction
from psaf_messages.msg import XRoute
from rosbag import ROSBagException

from psaf_scenario.scenario_runner_commonroad import ScenarioRunner


class ScenarioRunnerXRoute(ScenarioRunner):
    def __init__(self, route_file: str = "path.debugpath", init_rospy: bool = False, timeout: int = -1,
                 radius: int = 5, sample_cnt: int = 600, height: float = 1):
        super(ScenarioRunnerXRoute, self).__init__(route_file=route_file, init_rospy=init_rospy, timeout=timeout,
                                                   radius=radius, sample_cnt=sample_cnt, height=height)
        self.xroute_pub = rospy.Publisher('/psaf/xroute', XRoute, queue_size=10)
        self.x_route: XRoute = None

    def _unpack_route(self):
        """
        This function extracts the given route bag file and converts its content inits a list for further operations
        :return:
        """
        rospy.loginfo("ScenarioRunner: Starting the unpacking")
        try:
            bag = rosbag.Bag(self.route_file)
            for topic, msg, t in bag.read_messages(topics=['XRoute']):
                self.x_route = msg
                for xlanelet in msg.route:
                    for point in xlanelet.route_portion:
                        pose = PoseStamped(pose=Pose(position=Point(x=point.x, y=point.y, z=point.z)))
                        self.planned_route.append(pose)
            bag.close()
            for i in range(0, len(self.planned_route)):
                if i == 0:
                    yaw = self._orientation_for_point(self.planned_route[i+1].pose.position, self.planned_route[i].pose.position)
                elif i == len(self.planned_route)-1:
                    self.planned_route[i-1].pose.orientation = self.planned_route[i-1].pose.orientation
                    break
                else:
                    yaw = self._orientation_for_point(self.planned_route[i].pose.position, self.planned_route[i-1].pose.position)
                q = tf.transformations.quaternion_from_euler(0, 0, yaw)
                self.planned_route[i-1].pose.orientation = Quaternion(*q)
            rospy.loginfo("ScenarioRunner: Route unpacked")
        except ROSBagException:
            rospy.logerr("ScenarioRunner: Failure while extracting the route")
            sys.exit(-1)

    def _trigger_move_base(self, target: PoseStamped):
        """
        This function triggers the move_base by publishing the last entry in path, which is later used for sanity checking
        The last entry can be the goal if a path was found or the starting point if no path was found
        """
        while self.xroute_pub.get_num_connections() == 0:
            self.rate.sleep()
        self.xroute_pub.publish(self.x_route)
        # trigger move_base with dummy point
        client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        client.wait_for_server()
        target.header.frame_id = "map"
        goal = MoveBaseGoal()
        goal.target_pose = target

        client.send_goal(goal)

    def _orientation_for_point(self, pos, prev_pos):
        angles = list()
        # describes the relative position of the pos to the prev pos
        rel_x = 1 if (pos.x - prev_pos.x) >= 0 else -1
        rel_y = 1 if (prev_pos.y - pos.y) >= 0 else -1
        euler_angle_yaw = math.degrees(
            math.atan2(rel_y * abs(pos.y - prev_pos.y), rel_x * abs(pos.x - prev_pos.x)))
        # get orientation without multiple rotations
        euler_angle_yaw = euler_angle_yaw % 360
        # get the positive angle
        if euler_angle_yaw < 0:
            euler_angle_yaw = euler_angle_yaw + 360
        return math.radians(euler_angle_yaw)


def main():
    print(rospy.get_name())
    timeout = rospy.get_param('/scenario_runner_commonroad_xroute/timeout', -1.0)
    radius = rospy.get_param('/scenario_runner_commonroad_xroute/radius', 5)
    sample_cnt = rospy.get_param('/scenario_runner_commonroad_xroute/sample_cnt', 100)
    file = rospy.get_param('/scenario_runner_commonroad_xroute/file', 'path.debugpath')
    height = rospy.get_param('/scenario_runner_commonroad_xroute/height', 10)
    scenario = ScenarioRunnerXRoute(init_rospy=True, timeout=timeout, radius=radius, route_file=file,
                                    sample_cnt=sample_cnt, height=height)
    scenario.init_scenario()
    scenario.execute_scenario()
    scenario.evaluate_route_quality_cos()
    scenario.plot_routes()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
