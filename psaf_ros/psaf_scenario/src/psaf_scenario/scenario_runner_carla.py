#!/usr/bin/env python

import rosbag
import rospy
from rosbag import ROSBagException
from tf.transformations import euler_from_quaternion
from xml.etree.ElementTree import Element, SubElement, ElementTree
import os
import sys

from nav_msgs.msg import Path
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import PoseStamped, Point, Quaternion


class ScenarioRunner:
    def __init__(self, route: str = "", town: str = "Town03", init_rospy: bool = False, role_name: str = "ego_vehicle"):
        self.runner_root: str = ""
        if init_rospy:
            rospy.init_node('ScenarioRunner', anonymous=True)
        try:
            self.runner_root = os.environ['SCENARIO_RUNNER_ROOT']
        except KeyError:
            sys.exit(1)
        if route != "":
            self._gen_route_from_bag(route, town)

    def _gen_route_from_bag(self, route: str, town: str) -> bool:
        try:
            bag = rosbag.Bag(route)
            root = Element('routes')
            path = SubElement(root, 'route', {'id': "0", 'town': town})
            for topic, msg, t in bag.read_messages(topics=['Path']):
                for i in range(0, len(msg.poses), 100):
                    pose = msg.poses[i]
                    orientation_q = pose.pose.orientation
                    orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
                    (roll, pitch, yaw) = euler_from_quaternion(orientation_list)
                    waypoint = SubElement(path, 'waypoint',
                                          {'pitch': str(pitch),
                                           'roll': str(roll),
                                           'x': str(pose.pose.position.x),
                                           'y': str(pose.pose.position.y),
                                           'yaw': str(yaw),
                                           'z': str(pose.pose.position.z)})
            ElementTree(root).write('path.xml', method='xml')
            bag.close()
            return True
        except ROSBagException:
            return False

    def execute_scenario(self, scenario_file):
        """
        Executes scenario
        """
        cmdline = ["/usr/bin/python", "{}/scenario_runner.py".format(self.runner_root),
                   "--route", "{} {}".format('path.xml', 'asdf'),
                   "--agent", "{}/srunner/autoagents/ros_agent.py".format(self.runner_root),
                   "--timeout", "1000000",
                   "--host", self._host,
                   "--port", str(self._port)]
        if self._wait_for_ego:
            cmdline.append("--waitForEgo")
        return self.execute(cmdline, env=os.environ)

def main():
    scenario = ScenarioRunner(route='path.path', init_rospy=True)
    print(os.environ)


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
