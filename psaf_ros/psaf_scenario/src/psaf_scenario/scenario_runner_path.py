#!/usr/bin/env python
import sys

import actionlib
import rosbag
import rospy
from geometry_msgs.msg import PoseStamped
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from rosbag import ROSBagException

from psaf_scenario.scenario_runner_commonroad import ScenarioRunner


class ScenarioRunnerPath(ScenarioRunner):
    def __init__(self, route_file: str = "path.debugpath", init_rospy: bool = False, timeout: int = -1,
                 radius: int = 5, sample_cnt: int = 600, height: float = 1):
        super(ScenarioRunnerPath, self).__init__(route_file=route_file, init_rospy=init_rospy, timeout=timeout,
                                                 radius=radius, sample_cnt=sample_cnt, height=height);

    def _unpack_route(self):
        """
        This function extracts the given route bag file and converts its content inits a list for further operations
        :return:
        """
        rospy.loginfo("ScenarioRunner: Starting the unpacking")
        try:
            bag = rosbag.Bag(self.route_file)
            for topic, msg, t in bag.read_messages(topics=['Path']):
                for i in range(0, len(msg.poses)):
                    pose = msg.poses[i]
                    self.planned_route.append(pose)
            bag.close()
            rospy.loginfo("ScenarioRunner: Route unpacked")
        except ROSBagException:
            rospy.logerr("ScenarioRunner: Failure while extracting the route")
            sys.exit(-1)

    def _trigger_move_base(self, target: PoseStamped):
        """
        This function triggers the move_base by publishing the last entry in path, which is later used for sanity checking
        The last entry can be the goal if a path was found or the starting point if no path was found
        :param target: position and orientation of the goal
        :return:
        """
        client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        client.wait_for_server()

        goal = MoveBaseGoal()
        goal.target_pose = target

        client.send_goal(goal)


def main():
    print(rospy.get_name())
    timeout = rospy.get_param('/scenario_runner_commonroad_path/timeout', -1.0)
    radius = rospy.get_param('/scenario_runner_commonroad_path/radius', 5)
    sample_cnt = rospy.get_param('/scenario_runner_commonroad_path/sample_cnt', 100)
    file = rospy.get_param('/scenario_runner_commonroad_path/file', 'path.debugpath')
    height = rospy.get_param('/scenario_runner_commonroad_path/height', 10)
    scenario = ScenarioRunnerPath(init_rospy=True, timeout=timeout, radius=radius, route_file=file,
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
