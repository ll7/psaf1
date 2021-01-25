#!/usr/bin/env python

import rospy
from psaf_messages.msg import Obstacle
from geometry_msgs.msg import Point


def main():
    rospy.init_node('TEST', anonymous=True)
    obs_1 = Point(0, -10, 0)
    obs_3 = Point(0, 10, 0)
    ob = Obstacle(8, [obs_1, obs_3])
    pub = rospy.Publisher("/psaf/planning/obstacle", Obstacle, queue_size=10)
    while pub.get_num_connections() == 0:
        rospy.sleep(1)
    pub.publish(ob)


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
