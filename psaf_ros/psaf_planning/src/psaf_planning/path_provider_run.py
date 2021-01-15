#!/usr/bin/env python

from psaf_planning.global_planner.path_provider_lanelet2 import PathProviderLanelet2
from psaf_planning.global_planner.path_provider_abstract import PathProviderAbstract
from psaf_planning.global_planner.path_provider_common_roads import PathProviderCommonRoads
import rospy
from psaf_messages.msg import Obstacle


def main():
    rospy.init_node('TEST', anonymous=True)
    ob = Obstacle(1, 0, 0, -10, 0)
    pub = rospy.Publisher("/psaf/planning/obstacle", Obstacle, queue_size=10)
    while pub.get_num_connections() == 0:
        rospy.sleep(1)
    pub.publish(ob)


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
