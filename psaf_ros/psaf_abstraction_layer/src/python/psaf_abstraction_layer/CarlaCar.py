#!/usr/bin/env python

import rospy
from carla_msgs.msg import CarlaEgoVehicleControl
from ackermann_msgs.msg import AckermannDrive
from psaf_abstraction_layer.sensors.GPS import GPS_Sensor
from psaf_abstraction_layer.VehicleStatus import VehicleStatusProvider


def publish(publisher, message):
    rate = rospy.Rate(10)
    while publisher.get_num_connections() == 0:
        rate.sleep()
    publisher.publish(message)


class AckermannControl:
    """
    Wrapper for Ackermann controller of message of carla
    """

    def __init__(self, role_name: str = "ego_vehicle"):
        self.pub_ackermann = rospy.Publisher('/carla/{}/ackermann_cmd'.format(role_name), AckermannDrive, queue_size=1)
        self.message = AckermannDrive()

        # desired virtual angle (radians)

    def set_steering_angle(self, value: float):
        """
        Desired virtual angle in radians
        :param value: the new value
        :return: None
        """
        self.message.steering_angle = value
        self.__publish__()

    def set_steering_angle_velocity(self, value: float):
        """
         Desired rate of change (radians/s)
         :param value: the new value
         :return: None
         """
        self.message.steering_angle_velocity = value
        self.__publish__()

    def set_speed(self, value: float):
        """
         Desired forward speed (m/s)
         :param value: the new value
         :return: None
        """
        self.message.speed = value
        self.__publish__()

    def set_acceleration(self, value: float):
        """
        Desired acceleration (m/s^2)
        :param value: the new value
        :return: None
        """
        self.message.acceleration = value
        self.__publish__()

    def set_jerk(self, value: float):
        """
        Desired jerk (m/s^3)
        :param value: the new value
        :return: None
        """
        self.message.jerk = value
        self.__publish__()

    def __publish__(self):
        """
        Publish the current message
        :return: None
        """
        publish(self.pub_ackermann, self.message)

    def periodic_update(self, event):
        self.__publish__()


class Car:
    """
    Abstraction for a carla car
    """

    def __init__(self, role_name: str = "ego_vehicle", ackermann_active=False):
        """
        Constructor
        Attention: rospy.init_node(..) has to be called in advance
        :param role_name: the role name of the care default ist "ego_vehicle" 
        :param node: the node identifier
        :param ackermann_active: if you want to use the ackermann control you have to activate it here or later per method call
        """
        self.gps = GPS_Sensor(role_name)
        self.status_provider = VehicleStatusProvider(role_name)
        self.__car_cmd_msg = CarlaEgoVehicleControl()
        self.ackermann = AckermannControl(role_name)

        # Init publishers
        self.__pub_car_cmd = rospy.Publisher('/carla/{}/vehicle_control_cmd'.format(role_name), CarlaEgoVehicleControl,
                                             queue_size=2)

        # Periodic write of the message
        self.__timerCar = None
        self.__timerAcker = None
        self.activate_ackermann(ackermann_active)
        rospy.loginfo("Car abstraction init done")

    def activate_ackermann(self,value:bool):
        if not value:
            self.__timerCar = rospy.Timer(rospy.Duration(0.1), self.periodic_update)
            if not self.__timerAcker is None:
                self.__timerAcker.shutdown()
        else:
            self.__timerAcker = rospy.Timer(rospy.Duration(0.1), self.ackermann.periodic_update)
            if not self.__timerCar is None:
                self.__timerCar.shutdown()

    def set_throttle(self, value: float):
        """
        Set the throttle. 0. <= value <= 1.
        :param value: the new value
        :return: none
        """
        self.__car_cmd_msg.throttle = value
        self.__publish__()

    def set_steer(self, value: float):
        """
        Set the steering value. -1. <= value <= 1..
        :param value: the new value
        :return: None
        """
        self.__car_cmd_msg.steer = value
        self.__publish__()

    def set_brake(self, value: float):
        """
        Set the brake value. 0. <= value <= 1.
        :param value: the new value
        :return: None
        """
        self.__car_cmd_msg.brake = value
        self.__publish__()

    def set_hand_brake(self, value: bool):
        """
        Set whether the handbrake should be activated
        :param value: the new value
        :return: None
        """
        self.__car_cmd_msg.hand_brake = 1 if value else 0
        self.__publish__()

    def set_reverse(self, value: bool):
        """
        Set whether the gear is set to reverese
        :param value: the new value
        :return: None
        """
        self.__car_cmd_msg.reverse = 1 if value else 0
        self.__publish__()

    # gear
    def set_gear(self, value: int):
        """
        Set the desired gear as integer
        :param value: the new value
        :return: None
        """
        self.__car_cmd_msg.gear = value
        self.__publish__()

    # manual gear shift
    def set_manual_gear_shift(self, value: bool):
        """
        Set whether we want a manual or automatic gear shift
        :param value: the new value
        :return: None
        """
        self.__car_cmd_msg.manual_gear_shift = 1 if value else 0
        self.__publish__()

    def __publish__(self):
        """
        Publish the current message
        :return: None
        """
        if self.__timerAcker is None or not self.__timerAcker.is_alive():
            publish(self.__pub_car_cmd, self.__car_cmd_msg)

    def periodic_update(self, event):
        self.__publish__()

    def get_gps_sensor(self)->GPS_Sensor:
        """
        Returns the gps sensor attached to the car
        :return: the gps sensor
        """
        return self.gps

    def get_ackermann_control(self) -> AckermannControl:
        """
        Returns the ackerman control for the vehicle
        :return: the ackerman control
        """
        return self.ackermann

    def get_status_provider(self) -> VehicleStatusProvider:
        """
        Returns the vehicle status
        :return: the vehicle status
        """
        return self.status_provider