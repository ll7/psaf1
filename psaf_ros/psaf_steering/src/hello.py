#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from carla_msgs.msg import CarlaEgoVehicleControl

def talker():
    pub = rospy.Publisher('chatter', String, queue_size=10)
    pub_carla = rospy.Publisher('/carla/ego_vehicle/vehicle_control_cmd', CarlaEgoVehicleControl, queue_size=10)

    rospy.init_node('hello', anonymous=True)
    rate = rospy.Rate(10) # 10hz

    contr = CarlaEgoVehicleControl()
    contr.throttle = 1.0

    while not rospy.is_shutdown():
        hello_str = "hello world %s" % rospy.get_time()
        rospy.loginfo(hello_str)
        pub.publish(hello_str)
        pub_carla.publish(contr)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
