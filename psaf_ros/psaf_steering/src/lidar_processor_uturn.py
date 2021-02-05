#!/usr/bin/env python

from logging import info
import rospy
import math
from geometry_msgs.msg import Twist
from ackermann_msgs.msg import AckermannDrive
from rospy.core import rospyinfo
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.point_cloud2 import PointField
from std_msgs.msg import Header, Bool, Float32MultiArray
from nav_msgs.msg import Odometry



pub_distances = None
inner_sub = None
outer_sub = None
points = []


def publish_distances():
    print('publish')
    global pub_distances, points, inner_sub, outer_sub
    min_forward = sorted(points, key=lambda tup: tup[0])[0][0]
    min_left = sorted(points, key=lambda tup: tup[1])[0][1]
    print('min_left: ', min_left)
    print('min_forward: ', min_forward)

    p = points

    out = Float32MultiArray()
    out.data = [min_forward, min_left]

    pub_distances.publish(out)
    pub_distances = None
    points = []



def lidar_callback_outer(data: PointCloud2):
    print('lidar_callback_outer')
    global points, outer_sub
    for p in pc2.read_points(data, skip_nans=True):
        if p[5] in [1, 2, 5, 9, 11, 15, 16, 17, 18, 19, 20, 21]:
            if (p[0] > 2 ) and (p[1] > 1):
                points.append([abs(p[0]), abs(p[1]), p[5]])
    outer_sub.unregister()
    outer_sub = None
    if inner_sub is None and outer_sub is None:
        publish_distances()

def lidar_callback_inner(data: PointCloud2):
    print('lidar_callback_inner')
    global points, inner_sub
    for p in pc2.read_points(data, skip_nans=True):
        if p[5] in [1, 2, 5, 9, 11, 15, 16, 17, 18, 19, 20, 21]:
            if (p[0] > 2 ) and (p[1] > 1):
                points.append([abs(p[0]), abs(p[1]), p[5]])
    inner_sub.unregister()
    inner_sub = None
    if inner_sub is None and outer_sub is None:
        publish_distances()

def check_uturn(data: Bool):
    global pub_distances, inner_sub, outer_sub
    print('checking uturn')
    if pub_distances is None:
        outer_sub = rospy.Subscriber('/carla/ego_vehicle/semantic_lidar/lidar_outer/point_cloud', PointCloud2, lidar_callback_outer, queue_size=1)
        inner_sub = rospy.Subscriber('/carla/ego_vehicle/semantic_lidar/lidar_inner/point_cloud', PointCloud2, lidar_callback_inner, queue_size=1)
        pub_distances = rospy.Publisher('/pasf/lidar_processor/u_turn_distances', Float32MultiArray, queue_size=1)

if __name__ == '__main__':
    try:
        rospy.init_node('lidar_processor_uturn')

        rospy.Subscriber('/psaf/lidar_processor/check_u_turn', Bool, check_uturn, queue_size=1)

        rospy.spin()

    except rospy.ROSInterruptException:
        pass
