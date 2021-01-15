#!/usr/bin/env python
import rospy, math
from carla_msgs.msg import CarlaEgoVehicleControl, CarlaEgoVehicleStatus
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64
import cv2
import numpy as np
from psaf_abstraction_layer.sensors.RGBCamera import RGBCamera

from tf.transformations import euler_from_quaternion


# curvature estimation of local_plan
def callback_curvature(data):
    print('Curvature: ' + str(data) + ' °')


# control cmds that pid_controller sends to vehicle
def callback_cmd(data: CarlaEgoVehicleControl):
    global x1,y1,x2,y2
    # -1. <= steer <= 1.
    print('Steer: ' + str(data.steer))
    # 0. <= throttle <= 1.
    print('Throttle: ' + str(data.throttle))

    width = 0.34
    x1 = data.steer * 0.1 + 0.5

    # Calculate coordinates
    x2 = min([x1 + width,1])
    y1 = 0.2
    y2 = 0.8


# vehicle current status (speed, abs. orientation in world)
def callback_odom(data: CarlaEgoVehicleStatus):
    current_speed = data.velocity * (-1 if data.control.reverse else 1)
    print('Speed: ' + str(current_speed) + ' m/s')  # speed in m/s

    q = (data.orientation.x, data.orientation.y, data.orientation.z, data.orientation.w)
    _, _, yaw = euler_from_quaternion(q)
    orientation = math.degrees(yaw)
    print('Abs. Orientation: ' + str(orientation) + ' °')


# speed, angle setpoint from local_planner to pid_controller
def callback_setpoint(data: Twist):
    global x1,y1,x2,y2
    steering_angle = math.degrees(data.angular.z)
    print('Steering Angle Setpoint: ' + str(steering_angle) + ' °')
    speed_setpoint = data.linear.x
    print('Speed Setpoint: ' + str(speed_setpoint) + ' m/s')




# Show case code
def show_image(title, image):
    max_width, max_height = 1200, 800

    limit = (max_height, max_width)
    fac = 1.0
    if image.shape[0] > limit[0]:
        fac = limit[0] / image.shape[0]
    if image.shape[1] > limit[1]:
        fac = limit[1] / image.shape[1]
    image = cv2.resize(image, (int(image.shape[1] * fac), int(image.shape[0] * fac)))
    # show the output image
    cv2.imshow(title, cv2.cvtColor(image, cv2.COLOR_RGB2BGR))
    cv2.waitKey(1)


def show_rectangle(image,_):
    global x1,y1,x2,y2
    H, W = image.shape[:2]


    # extract the bounding box coordinates
    coords = np.array([x1, y1, x2, y2]) * np.array([W, H, W, H])
    abs_x1, abs_y1, abs_x2, abs_y2 = coords.astype(int)
    # draw a bounding box rectangle and label on the image
    color = (255, 0, 0)
    cv2.rectangle(image, (abs_x1, abs_y1), (abs_x2, abs_y2), color, 2)

    # show the output image
    show_image("Detection Area", image)


def listener():
    global x1,y1,x2,y2
    rospy.init_node('listener', anonymous=True)

    rospy.Subscriber("/pasf/local_planner/curvature", Float64, callback_curvature)
    rospy.Subscriber("/carla/ego_vehicle/vehicle_control_cmd", CarlaEgoVehicleControl, callback_cmd)
    rospy.Subscriber("/carla/ego_vehicle/vehicle_status", CarlaEgoVehicleStatus, callback_odom)
    rospy.Subscriber("/carla/ego_vehicle/twist_pid", Twist, callback_setpoint)

    cam = RGBCamera()

    x1, y1, x2, y2 = 0,0,0,0
    cam.set_on_image_listener(show_rectangle)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()


if __name__ == '__main__':
    listener()
