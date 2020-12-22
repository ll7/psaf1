#!/usr/bin/env python

import rospy
import math
from geometry_msgs.msg import Twist
from ackermann_msgs.msg import AckermannDrive
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.point_cloud2 import PointField
from std_msgs.msg import Header
from nav_msgs.msg import Odometry


zPos = 0
def odo_callback(data: Odometry):
    zPos = data.pose.pose.position.z


def cmd_callback(data: PointCloud2):
    # field_names=("x", "y", "z", "cos", "index", "tag")
    skip = 0
    skip2 = 0

    points = []
    for p in pc2.read_points(data, skip_nans=True):
        # print("%f %f %f: %i", p[0], p[1], p[2], p[3], p[4], p[5])
        if (p[5] == 6 or p[5] == 7):
            skip += 1
            continue
        if (abs(p[0]) < 2.0 and abs(p[1]) < 1.0):
            skip2 += 2
            continue

        points.append([p[0], p[1], p[2] - zPos])
    
    # https://gist.github.com/lucasw/ea04dcd65bc944daea07612314d114bb

    fields = [PointField('x', 0, PointField.FLOAT32, 1),
          PointField('y', 4, PointField.FLOAT32, 1),
          PointField('z', 8, PointField.FLOAT32, 1),
          # PointField('rgb', 12, PointField.UINT32, 1),
          # PointField('rgba', 12, PointField.UINT32, 1),
          ]

    header = Header()
    header.frame_id = "ego_vehicle/lidar/lidar1"
    header.stamp = rospy.Time.now()
    cloud = pc2.create_cloud(header, fields, points)
    pub.publish(cloud)


if __name__ == '__main__':
    try:

        rospy.init_node('semantic_lidar_processor')

        rospy.Subscriber("/carla/ego_vehicle/semantic_lidar/lidar1/point_cloud/", PointCloud2, cmd_callback, queue_size=1)
        rospy.Subscriber("/carla/ego_vehicle/odometry", Odometry, odo_callback, queue_size=1)
        pub = rospy.Publisher("/carla/ego_vehicle/processed_semantic_lidar/lidar1/point_cloud/", PointCloud2, queue_size=1)

        # rospy.loginfo("Node 'semantic_lidar_processor' started.\nListening to %s, publishing to %s. Frame id: %s, wheelbase: %f", "/cmd_vel", ackermann_cmd_topic, frame_id, wheelbase)

        rospy.spin()

    except rospy.ROSInterruptException:
        pass
