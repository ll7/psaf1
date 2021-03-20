import rospy
from carla_msgs.msg import CarlaEgoVehicleStatus
from geometry_msgs.msg import Quaternion,Accel
from tf.transformations import euler_from_quaternion

class VehicleStatus:
    """
    Vehicle Status
    Attr:
        orientation     the orientation
        acceleration    the acceleration as geometry_msgs.msg.Accel
        velocity        the velocity in m/s
    """


    def __init__(self, orientation: Quaternion = Quaternion(), acceleration: Accel = Accel(), velocity: float = 0.):

        self.orientation = orientation
        self.acceleration = acceleration
        self.velocity = velocity

    def __str__(self) -> str:
        return "orientation:'{}'-acc:'{}'-vel:'{}m/s'".format(self.orientation, self.acceleration, self.velocity)

    def get_orientation_as_euler(self):
        """
        Returns the quanterian as roll pitch yaw
        :return: [0]= roll, [1] = pitch, [2] = yaw
        """
        return euler_from_quaternion((self.orientation.x,self.orientation.y,self.orientation.z,self.orientation.w))


class VehicleStatusProvider:
    """
    Offers the information about the carla ego vehicle
    """

    def __init__(self, role_name: str):
        """
        Constructor
        :param role_name: the role name of the vehicle
        """

        self.status = VehicleStatus()
        self.__subscriber = rospy.Subscriber("/carla/{}/vehicle_status".format(role_name), CarlaEgoVehicleStatus,
                                             self.__update_status)
        self.__listener = None
        self.status_available: bool = False

    def __update_status(self,message:CarlaEgoVehicleStatus):
        self.status = VehicleStatus(message.orientation,message.acceleration,message.velocity)
        self.status_available = True
        if self.__listener != None:
            self.__listener(self.status)

    def set_on_status_update_listener(self, func):
        """
        Set function to be called with the new VehicleStatus as parameter
        :param func: the function
        :return: None
        """
        self.__listener = func

    def get_status(self) -> VehicleStatus:
        """
        Return the current status
        :return: the current status
        """
        return self.status
