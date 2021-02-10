#!/usr/bin/env python
import rospy, math
from carla_msgs.msg import CarlaEgoVehicleControl, CarlaEgoVehicleStatus
from geometry_msgs.msg import Twist
from psaf_messages.msg import TrafficSignInfo
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
    global steer
    steer = data.steer




# speed, angle setpoint from local_planner to pid_controller
def callback_setpoint(data: Twist):
    global x1,y1,x2,y2
    steering_angle = math.degrees(data.angular.z)
    # print('Steering Angle Setpoint: ' + str(steering_angle) + ' °')
    speed_setpoint = data.linear.x
    # print('Speed Setpoint: ' + str(speed_setpoint) + ' m/s')


def calculate_rects():
    # Copy the value to work always with he same data
    global steer,traffic_lights,rectangles

    for each in traffic_lights:

    # check on right hand side for traffic lights (e.g. in small towns like town02)
    # if traffic_light.distance < 50:
    #     x1 = steer * (0.3 if steer > 0 else 0.6) + 0.5
    #     # Calculate coordinates
    #     x2 = min([x1 + 0.4, 1.])
    #     first_check = is_in_traffic_light_area(traffic_light, x1, x2, 0.4, 0.8)
    #     second_check = traffic_light.x > 0.7 and traffic_light.distance < 3  # If we are very close to the traffic light
    #     if first_check or second_check:
    #         return True
    # check for traffic lights that are in front of the car on the other side of the intersection (american style)


        # scale width and height by distance -> size decrease with distance
        scaling = 1 - math.pow(4, -(each.distance) / 100)
        center_x = steer * (0.25 if steer > 0 else 0.3) + 0.5
        center_y = 0.375 + 0.1*scaling
        width = 0.3 * scaling
        height = 0.25 * scaling
        rectangles.append( np.array([center_x - width / 2, center_x + width / 2,
                                        center_y - height / 2, center_y + height / 2]))



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
    global rectangles
    H, W = image.shape[:2]
    calculate_rects()

    for  rectangle in rectangles:
        # extract the bounding box coordinates
        coords = rectangle * np.array([W, W,H, H])
        abs_x1,  abs_x2, abs_y1, abs_y2 = coords.astype(int)
        # draw a bounding box rectangle and label on the image
        color = (255, 0, 0)
        cv2.rectangle(image, (abs_x1, abs_y1), (abs_x2, abs_y2), color, 2)
    rectangles.clear()
    # show the output image
    show_image("Detection Area", image)

def callback_traffic_signs(msg:TrafficSignInfo):
    global traffic_lights
    traffic_lights.clear()
    traffic_lights.extend(msg.trafficLights)



def listener():
    global traffic_lights,rectangles,steer
    rospy.init_node('listener', anonymous=True)

    rospy.Subscriber("/psaf/local_planner/curvature", Float64, callback_curvature)
    rospy.Subscriber("/carla/ego_vehicle/vehicle_control_cmd", CarlaEgoVehicleControl, callback_cmd)
    rospy.Subscriber("/carla/ego_vehicle/twist_pid", Twist, callback_setpoint)
    rospy.Subscriber("/psaf/perception/traffic_signs", TrafficSignInfo, callback_traffic_signs)

    cam = RGBCamera()

    steer = 0.0
    traffic_lights=[]
    rectangles = []


    x1, y1, x2, y2 = 0,0,0,0
    center_x,center_y = 0,0
    cam.set_on_image_listener(show_rectangle)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()


if __name__ == '__main__':
    listener()
