#!/usr/bin/env python

import rospy
import numpy as np
from std_msgs.msg import String
from carla_msgs.msg import CarlaEgoVehicleControl
from sensor_msgs.msg import Image


#def normalize_meter(x):
#    normalized = (x[0] + x[1] * 256 + x[2] * 256 * 256) / (256 * 256 * 256 - 1)
#    in_meters = 1000 * normalized
#    return in_meters
#
#v_normalize_meter = np.vectorize(normalize_meter)
#
#
#def on_view_image(image: Image): 
#    array = np.frombuffer(image.data, dtype=np.dtype("uint8"))
#    array = np.reshape(array, (image.height, image.width, 4))
#    
#    v_normalize_meter()
#
#    pass

def talker():
    pub = rospy.Publisher('chatter', String, queue_size=10)
    role_name = "ego_vehicle"
    pub_carla = rospy.Publisher('/carla/ego_vehicle/vehicle_control_cmd', CarlaEgoVehicleControl, queue_size=10)

    # image_subscriber = rospy.Subscriber("/carla/{}/camera/rgb/view/image_color".format(role_name), Image, on_view_image)

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
