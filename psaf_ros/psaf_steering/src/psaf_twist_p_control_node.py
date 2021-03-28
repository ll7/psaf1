#!/usr/bin/env python

"""
Control Carla ego vehicle by using AckermannDrive messages
"""
import sys
import datetime
from collections import deque
from threading import active_count

import numpy
import rospy

from simple_pid import PID
import matplotlib.pyplot as plt

from dynamic_reconfigure.server import Server
from ackermann_msgs.msg import AckermannDrive
from carla_msgs.msg import CarlaEgoVehicleStatus
from carla_msgs.msg import CarlaEgoVehicleControl
from carla_msgs.msg import CarlaEgoVehicleInfo
from carla_ackermann_control.msg import EgoVehicleControlInfo
from psaf_steering.cfg import EgoVehicleControlParameterConfig
from geometry_msgs.msg import Twist
import carla_control_physics as phys


# Alternative way of plotting: 
# rqt_plot /carla/ego_vehicle/ackermann_control/control_info/target/steering_angle:speed carla/ego_vehicle/ackermann_control/control_info/current/speed carla/ego_vehicle/ackermann_control/control_info/output/steer:brake


class CarlaPControl(object):

    """
    Convert ackermann_drive messages to carla VehicleCommand with a PID controller
    """

    def __init__(self):
        """
        Constructor

        """
        self.frequency = rospy.get_param('~frequency', 20)
        self.control_loop_rate = rospy.Rate(self.frequency)  # 10Hz
        self.lastAckermannMsgReceived = datetime.datetime(datetime.MINYEAR, 1, 1)
        self.vehicle_status = CarlaEgoVehicleStatus()
        self.vehicle_info = CarlaEgoVehicleInfo()
        self.role_name = rospy.get_param('~role_name', 'ego_vehicle')
        # control info
        self.info = EgoVehicleControlInfo()

        # set initial maximum values
        self.vehicle_info_updated(self.vehicle_info)

        # target values
        self.info.target.steering_angle = 0.
        self.info.target.speed = 0.
        self.info.target.speed_abs = 0.
        self.info.target.accel = 0.
        self.info.target.jerk = 0.

        # current values
        self.info.current.time_sec = rospy.get_rostime().to_sec()
        self.info.current.speed = 0.
        self.info.current.speed_abs = 0.
        self.info.current.accel = 0.

        # control values
        self.info.status.status = 'n/a'
        self.info.status.speed_control_activation_count = 0
        self.info.status.speed_control_accel_delta = 0.
        self.info.status.speed_control_accel_target = 0.
        self.info.status.accel_control_pedal_delta = 0.
        self.info.status.accel_control_pedal_target = 0.
        self.info.status.brake_upper_border = 0.
        self.info.status.throttle_lower_border = 0.

        # control output
        self.info.output.throttle = 0.
        self.info.output.brake = 1.0
        self.info.output.steer = 0.
        self.info.output.reverse = False
        self.info.output.hand_brake = True

        # input parameters
        self.input_speed = 0.
        self.input_accel = 0.
        self.input_multiplier = 1.0

        # ackermann drive commands
        self.control_subscriber = rospy.Subscriber(
            "/carla/" + self.role_name + "/ackermann_cmd",
            AckermannDrive, self.ackermann_command_updated)

        #ackermann drive commands
        self.control_subscriber = rospy.Subscriber(
             "/carla/" + self.role_name + "/twist_pid",
             Twist, self.twist_command_updated)

        # current status of the vehicle
        self.vehicle_status_subscriber = rospy.Subscriber(
            "/carla/" + self.role_name + "/vehicle_status",
            CarlaEgoVehicleStatus, self.vehicle_status_updated)

        # vehicle info
        self.vehicle_info_subscriber = rospy.Subscriber(
            "/carla/" + self.role_name + "/vehicle_info",
            CarlaEgoVehicleInfo, self.vehicle_info_updated)

        # to send command to carla
        self.carla_control_publisher = rospy.Publisher(
            "/carla/" + self.role_name + "/vehicle_control_cmd",
            CarlaEgoVehicleControl, queue_size=1)

        # report controller info
        self.control_info_publisher = rospy.Publisher(
            "/carla/" + self.role_name + "/ackermann_control/control_info",
            EgoVehicleControlInfo, queue_size=1)


    def vehicle_status_updated(self, vehicle_status):
        """
        Stores the ackermann drive message for the next controller calculation

        :param ros_ackermann_drive: the current ackermann control input
        :type ros_ackermann_drive: ackermann_msgs.AckermannDrive
        :return:
        """

        # set target values
        self.vehicle_status = vehicle_status

    def vehicle_info_updated(self, vehicle_info):
        """
        Stores the ackermann drive message for the next controller calculation

        :param ros_ackermann_drive: the current ackermann control input
        :type ros_ackermann_drive: ackermann_msgs.AckermannDrive
        :return:
        """
        # set target values
        self.vehicle_info = vehicle_info

        # calculate restrictions
        self.info.restrictions.max_steering_angle = phys.get_vehicle_max_steering_angle(
            self.vehicle_info)
        self.info.restrictions.max_speed = phys.get_vehicle_max_speed(
            self.vehicle_info)
        self.info.restrictions.max_accel = phys.get_vehicle_max_acceleration(
            self.vehicle_info)
        self.info.restrictions.max_decel = phys.get_vehicle_max_deceleration(
            self.vehicle_info)
        self.info.restrictions.min_accel = rospy.get_param('/carla/ackermann_control/min_accel', 1.)
        # clipping the pedal in both directions to the same range using the usual lower
        # border: the max_accel to ensure the the pedal target is in symmetry to zero
        self.info.restrictions.max_pedal = min(
            self.info.restrictions.max_accel, self.info.restrictions.max_decel)

    def ackermann_command_updated(self, ros_ackermann_drive):
        """
        Stores the ackermann drive message for the next controller calculation

        :param ros_ackermann_drive: the current ackermann control input
        :type ros_ackermann_drive: ackermann_msgs.AckermannDrive
        :return:
        """
        self.lastAckermannMsgReceived = datetime.datetime.now()
        # set target values
        self.set_target_steering_angle(ros_ackermann_drive.steering_angle * self.input_multiplier)
        self.set_target_speed(ros_ackermann_drive.speed * self.input_multiplier)
        self.set_target_accel(ros_ackermann_drive.acceleration * self.input_multiplier)
        self.set_target_jerk(ros_ackermann_drive.jerk * self.input_multiplier)

        self.input_speed = ros_ackermann_drive.speed * self.input_multiplier
        self.input_accel = ros_ackermann_drive.acceleration * self.input_multiplier

    def twist_command_updated(self, ros_twist: Twist):
        """
        Stores the twist  message for the next controller calculation

        :param ros_twist: the current twist control input
        :type ros_twist: geometry_msgs.Twist
        :return:
        """
        self.lastAckermannMsgReceived = datetime.datetime.now()
        # set target values
        self.set_target_steering_angle(ros_twist.angular.z * self.input_multiplier)
        self.set_target_speed(ros_twist.linear.x * self.input_multiplier)
        self.set_target_accel(0)
        self.set_target_jerk(0)

        self.input_speed = ros_twist.linear.x * self.input_multiplier
        self.input_accel = 0.

    def reload_input_params(self):
        self.set_target_speed(self.input_speed)
        self.set_target_accel(self.input_accel)

    def set_target_steering_angle(self, target_steering_angle):
        """
        set target sterring angle
        """
        self.info.target.steering_angle = -target_steering_angle
        if abs(self.info.target.steering_angle) > self.info.restrictions.max_steering_angle:
            rospy.logerr_throttle(10, "Max steering angle reached, clipping value")
            self.info.target.steering_angle = numpy.clip(
                self.info.target.steering_angle,
                -self.info.restrictions.max_steering_angle,
                self.info.restrictions.max_steering_angle)

    def set_target_speed(self, target_speed):
        """
        set target speed
        """
        if abs(target_speed) > self.info.restrictions.max_speed:
            rospy.logerr_throttle(10, "Max speed reached, clipping value")
            
            self.info.target.speed = numpy.clip(
                target_speed, -self.info.restrictions.max_speed, self.info.restrictions.max_speed)
        else:
            self.info.target.speed = target_speed
        self.info.target.speed_abs = abs(self.info.target.speed)

    def set_target_accel(self, target_accel):
        """
        set target accel
        """
        epsilon = 0.00001
        # if speed is set to zero, then use max decel value
        if self.info.target.speed_abs < epsilon:
            self.info.target.accel = -self.info.restrictions.max_decel
        else:
            self.info.target.accel = numpy.clip(
                target_accel, -self.info.restrictions.max_decel, self.info.restrictions.max_accel)

    def set_target_jerk(self, target_jerk):
        """
        set target accel
        """
        self.info.target.jerk = target_jerk

    def override_speed(self):
        """
        fully breaks or fully stops when the diff is greater than x
        """
        diff_factor_throttle = 1.6
        diff_factor_break = 5

        diff = self.info.target.speed - self.info.current.speed

        if (diff > 0 ):
            self.info.output.throttle = min(diff / diff_factor_throttle, 1.0)
            self.info.output.brake = 0.0

        if (-diff >= 0):
            self.info.output.brake = min(-diff / diff_factor_break, 1.0)
            self.info.output.throttle = 0.0

    def vehicle_control_cycle(self):
        """
        Perform a vehicle control cycle and sends out CarlaEgoVehicleControl message
        """
        # reload the input parameters if the other node doesn't publish fast enough
        self.reload_input_params()

        # perform actual control
        self.control_steering()
        self.control_stop_and_reverse()
        if not self.info.output.hand_brake:
            self.update_drive_vehicle_control_command()

            self.override_speed()
            self.carla_control_publisher.publish(self.info.output)

    def control_steering(self):
        """
        Basic steering control
        """
        self.info.output.steer = self.info.target.steering_angle / self.info.restrictions.max_steering_angle

    def control_stop_and_reverse(self):
        """
        Handle stop and switching to reverse gear
        """
        # from this velocity on it is allowed to switch to reverse gear
        standing_still_epsilon = 0.1
        # from this velocity on hand brake is turned on
        full_stop_epsilon = 0.01

        # auto-control of hand-brake and reverse gear
        self.info.output.hand_brake = False
        if self.info.current.speed_abs < standing_still_epsilon:
            # standing still, change of driving direction allowed
            self.info.status.status = "standing"
            if self.info.target.speed < 0:
                if not self.info.output.reverse:
                    rospy.loginfo(
                        "VehicleControl: Change of driving direction to reverse")
                    self.info.output.reverse = True
            elif self.info.target.speed > 0:
                if self.info.output.reverse:
                    rospy.loginfo(
                        "VehicleControl: Change of driving direction to forward")
                    self.info.output.reverse = False
            if self.info.target.speed_abs < full_stop_epsilon:
                self.info.status.status = "full stop"
                self.info.status.speed_control_accel_target = 0.
                self.info.status.accel_control_pedal_target = 0.
                self.set_target_speed(0.)
                self.info.current.speed = 0.
                self.info.current.speed_abs = 0.
                self.info.current.accel = 0.
                self.info.output.hand_brake = True
                self.info.output.brake = 1.0
                self.info.output.throttle = 0.0

        elif numpy.sign(self.info.current.speed) * numpy.sign(self.info.target.speed) == -1:
            # requrest for change of driving direction
            # first we have to come to full stop before changing driving
            # direction
            rospy.loginfo("VehicleControl: Request change of driving direction."
                          " v_current={} v_desired={}"
                          " Set desired speed to 0".format(self.info.current.speed,
                                                           self.info.target.speed))
            self.set_target_speed(0.)
            self.set_target_accel(0.)

    def update_drive_vehicle_control_command(self):
        """
        Apply the current speed_control_target value to throttle/brake commands
        """

        # the driving impedance moves the 'zero' acceleration border
        # Interpretation: To reach a zero acceleration the throttle has to pushed
        # down for a certain amount
        self.info.status.throttle_lower_border = phys.get_vehicle_driving_impedance_acceleration(
            self.vehicle_info, self.vehicle_status, self.info.output.reverse)

        # the engine lay off acceleration defines the size of the coasting area
        # Interpretation: The engine already prforms braking on its own;
        #  therefore pushing the brake is not required for small decelerations
        self.info.status.brake_upper_border = self.info.status.throttle_lower_border + \
            phys.get_vehicle_lay_off_engine_acceleration(self.vehicle_info)

        if self.info.status.accel_control_pedal_target > self.info.status.throttle_lower_border:
            self.info.status.status = "accelerating"
            self.info.output.brake = 0.0
            # the value has to be normed to max_pedal
            # be aware: is not required to take throttle_lower_border into the scaling factor,
            # because that border is in reality a shift of the coordinate system
            # the global maximum acceleration can practically not be reached anymore because of
            # driving impedance
            self.info.output.throttle = (
                (self.info.status.accel_control_pedal_target -
                 self.info.status.throttle_lower_border) /
                abs(self.info.restrictions.max_pedal))
        elif self.info.status.accel_control_pedal_target > self.info.status.brake_upper_border:
            self.info.status.status = "coasting"
            # no control required
            self.info.output.brake = 0.0
            self.info.output.throttle = 0.0
        else:
            self.info.status.status = "braking"
            # braking required
            self.info.output.brake = (
                (self.info.status.brake_upper_border -
                 self.info.status.accel_control_pedal_target) /
                abs(self.info.restrictions.max_pedal))
            self.info.output.throttle = 0.0

        # finally clip the final control output (should actually never happen)
        self.info.output.brake = numpy.clip(self.info.output.brake, 0., 1.)
        self.info.output.throttle = numpy.clip(self.info.output.throttle, 0., 1.)

    # from ego vehicle
    def send_ego_vehicle_control_info_msg(self):
        """
        Function to send carla_ackermann_control.msg.EgoVehicleControlInfo message.

        :return:
        """
        self.info.output.header = self.info.header
        self.info.output.header.stamp = rospy.Time.now()
        self.control_info_publisher.publish(self.info)


    def update_current_values(self):
        """
        Function to update vehicle control current values.

        we calculate the acceleration on ourselves, because we are interested only in
        the acceleration in respect to the driving direction
        In addition a small average filter is applied

        :return:
        """
        current_time_sec = rospy.get_rostime().to_sec()
        delta_time = current_time_sec - self.info.current.time_sec
        # rospy.loginfo("delta time: " + str(delta_time))

        current_speed = self.vehicle_status.velocity  * (-1 if self.vehicle_status.control.reverse else 1)
        if delta_time > 0:
            delta_speed = abs(current_speed) - abs(self.info.current.speed)
            # rospy.loginfo("delta speed: " + str(delta_speed))

            current_accel = delta_speed / delta_time
            # average filter
            self.info.current.accel = (self.info.current.accel * 9 + current_accel) / 10
            # rospy.loginfo("acc: " + str(self.info.current.accel))
        self.info.current.time_sec = current_time_sec
        self.info.current.speed = current_speed
        self.info.current.speed_abs = abs(current_speed)

    def run(self):
        """

        Control loop

        :return:
        """

        while not rospy.is_shutdown():
            self.update_current_values()
            self.vehicle_control_cycle()
            self.send_ego_vehicle_control_info_msg()
            try:
                self.control_loop_rate.sleep()
            except rospy.ROSInterruptException:
                pass


def main():
    """

    main function

    :return:
    """
    rospy.init_node('carla_ackermann_control', anonymous=True)
    controller = CarlaPControl()
    try:
        controller.run()
    finally:
        del controller
        rospy.loginfo("Done")


if __name__ == "__main__":
    main()
