#!/usr/bin/env python
import rospy, math
from carla_msgs.msg import CarlaEgoVehicleControl, CarlaEgoVehicleStatus
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64
from tf.transformations import euler_from_quaternion



# curvature estimation of local_plan
def callback_curvature(data):
    print('Curvature: ' + str(data) + ' °')

# control cmds that pid_controller sends to vehicle
def callback_cmd(data: CarlaEgoVehicleControl):
    # -1. <= steer <= 1.
    print('Steer: ' + str(data.steer))
    # 0. <= throttle <= 1.
    print('Throttle: ' + str(data.throttle))

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
    steering_angle = math.degrees(data.angular.z)
    print('Steering Angle Setpoint: ' + str(steering_angle) + ' °')
    speed_setpoint = data.linear.x
    print('Speed Setpoint: ' + str(speed_setpoint) + ' m/s')



def listener():
    rospy.init_node('listener', anonymous=True)

    rospy.Subscriber("/pasf/local_planner/curvature", Float64, callback_curvature)
    rospy.Subscriber("/carla/ego_vehicle/vehicle_control_cmd", CarlaEgoVehicleControl, callback_cmd)
    rospy.Subscriber("/carla/ego_vehicle/vehicle_status", CarlaEgoVehicleStatus, callback_odom)
    rospy.Subscriber("/carla/ego_vehicle/twist_pid", Twist, callback_setpoint)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()


if __name__ == '__main__':
    listener()