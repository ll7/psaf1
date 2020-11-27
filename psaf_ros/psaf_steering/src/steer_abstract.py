#!/usr/bin/env python

import rospy
#from psaf_ros.psaf_abstraction_layer.src.psaf_abstraction_layer.CarlaCar import Car
from psaf_abstraction_layer.CarlaCar import Car


if __name__ == '__main__':
    try:
        rospy.init_node('hello', anonymous=True)

        c = Car()

        rate = rospy.Rate(10)  # 10hz

        # while not rospy.is_shutdown():
        c.ackermann.set_speed(10.)
        c.ackermann.set_acceleration(10.)
        c.get_gps_sensor().set_on_position_listener(rospy.loginfo)
        #   rate.sleep()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
