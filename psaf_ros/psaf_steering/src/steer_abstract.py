#!/usr/bin/env python

import rospy
from math import pi
from psaf_abstraction_layer.CarlaCar import Car

def print_orientation(status):
    euler = status.get_orientation_as_euler()
    rospy.loginfo("roll:{} pitch:{} yaw:{}".format(euler[0],euler[1],euler[2]))

if __name__ == '__main__':
    try:
        rospy.init_node('hello', anonymous=True)

        c = Car(ackermann_active=True)

        rate = rospy.Rate(1)  # 10hz

        # while not rospy.is_shutdown():
        c.ackermann.set_speed(10.)
        c.ackermann.set_acceleration(10.)
        c.ackermann.set_steering_angle(pi/8)
        #c.get_gps_sensor().set_on_position_listener(rospy.loginfo)
        #c.get_status_provider().set_on_status_update_listener(print_orientation)
        while not rospy.is_shutdown():
            print_orientation(c.get_status_provider().status)
            rate.sleep()

    except rospy.ROSInterruptException:
        pass
