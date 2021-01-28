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
from std_msgs.msg import Header
from nav_msgs.msg import Odometry


zPos = 0
def odo_callback(data: Odometry):
    global zPos
    zPos = data.pose.pose.position.z + 3


def cmd_callback(data: PointCloud2):
    # field_names=("x", "y", "z", "cos", "index", "tag")
    skip = 0
    skip2 = 0

    points_marking = []
    points_clearing = []
    for p in pc2.read_points(data, skip_nans=True):
        # print("%f %f %f: %i", p[0], p[1], p[2], p[3], p[4], p[5])
        if (p[5] not in [4, 10]):
            # skip += 1
            points_clearing.append([p[0], p[1], p[2]])
            continue
        if (abs(p[0]) < 2.0 and abs(p[1]) < 1.0):
            skip2 += 2
            continue

        # points.append([p[0], p[1], p[2] - zPos])
        points_marking.append([p[0], p[1], p[2]])

    
    # https://gist.github.com/lucasw/ea04dcd65bc944daea07612314d114bb

    fields = [PointField('x', 0, PointField.FLOAT32, 1),
          PointField('y', 4, PointField.FLOAT32, 1),
          PointField('z', 8, PointField.FLOAT32, 1),
          # PointField('rgb', 12, PointField.UINT32, 1),
          # PointField('rgba', 12, PointField.UINT32, 1),
          ]


    cloud_marking = pc2.create_cloud(data.header, fields, points_marking)
    cloud_clearing = pc2.create_cloud(data.header, fields, points_clearing)
    pub_marking.publish(cloud_marking)
    pub_clearing.publish(cloud_clearing)


if __name__ == '__main__':
    try:
        

        rospy.init_node('semantic_lidar_processor')
        # print(rospy.get_name())
        # rospy.init_node(rospy.get_name())        

        rospy.Subscriber(rospy.get_param("~lidar_topic"), PointCloud2, cmd_callback, queue_size=1)
        rospy.Subscriber(rospy.get_param("~odom_topic"), Odometry, odo_callback, queue_size=1)
        pub_marking = rospy.Publisher(rospy.get_param("~marking_topic"), PointCloud2, queue_size=1)
        pub_clearing = rospy.Publisher(rospy.get_param("~clearing_topic"), PointCloud2, queue_size=1)

        # rospy.loginfo("Node 'semantic_lidar_processor' started.\nListening to %s, publishing to %s. Frame id: %s, wheelbase: %f", "/cmd_vel", ackermann_cmd_topic, frame_id, wheelbase)

        rospy.spin()

    except rospy.ROSInterruptException:
        pass
