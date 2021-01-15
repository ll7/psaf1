#!/usr/bin/env python

import rospy
from psaf_messages.msg import TrafficSignInfo, SpeedSign
from typing import List

rospy.init_node("publish_speedlimit_test")


def publish():
    pub = rospy.Publisher('/psaf/perception/traffic_signs/', TrafficSignInfo, queue_size=1)

    msg = TrafficSignInfo()
    msg.header.stamp = rospy.Time.now()
    msg.header.frame_id = "DetectionServiceTrafficSigns"

    speed_signs: List[SpeedSign] = []

    sign = SpeedSign()
    sign.x = 80
    sign.y = 80
    sign.distance = 5.0



    r = rospy.Rate(10)  # 10hz

    while not rospy.is_shutdown():
        value = input("Please enter limit: ")

        sign.limit = int(value)

        speed_signs.clear()
        speed_signs.append(sign)
        msg.speedSigns.extend(speed_signs)
        msg.header.stamp = rospy.Time.now()

        pub.publish(msg)

        r.sleep()


if __name__ == '__main__':
    try:
        publish()
    except rospy.ROSInterruptException:
        pass