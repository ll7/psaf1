#!/usr/bin/env python

import rospy
import numpy as np
from std_msgs.msg import String
from carla_msgs.msg import CarlaEgoVehicleControl
from sensor_msgs.msg import Image
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

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
    pub_viapoints = rospy.Publisher('/move_base/TebLocalPlannerROS/via_points', Path, queue_size=10)
    client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
    client.wait_for_server()

    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "ego_vehicle"
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose.position.x = 2.0
    goal.target_pose.pose.orientation.w = 1.0


    client.send_goal(goal)
    wait = client.wait_for_result()
    if not wait:
        rospy.logerr("Action server not available!")
        rospy.signal_shutdown("Action server not available!")
    else:
        rospy.loginfo(client.get_result())

    # image_subscriber = rospy.Subscriber("/carla/{}/camera/rgb/view/image_color".format(role_name), Image, on_view_image)

    rate = rospy.Rate(10) # 10hz

    contr = CarlaEgoVehicleControl()
    contr.throttle = 1.0

    # Try to publish atleast some nonsense viapoints
    via_points_msg = Path() 
    via_points_msg.header.stamp = rospy.Time.now()
    via_points_msg.header.frame_id = "map" # CHANGE HERE: odom/map
    
    # Add via-points
    point1 = PoseStamped()
    point1.pose.position.x = 0.0
    point1.pose.position.y = 0.0

    point2 = PoseStamped()
    point2.pose.position.x = 50.0
    point2.pose.position.y = 50.0


    via_points_msg.poses = [point1, point2]


    while not rospy.is_shutdown():
        hello_str = "hello world %s" % rospy.get_time()
        rospy.loginfo(hello_str)
        pub.publish(hello_str)
        # pub_carla.publish(contr)
        pub_viapoints.publish(via_points_msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        rospy.init_node('hello', anonymous=True)
        talker()
    except rospy.ROSInterruptException:
        pass
