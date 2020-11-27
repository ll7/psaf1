import rospy
from sensor_msgs.msg import NavSatFix, Image, PointCloud2


class GPS_Position:
    """
    Position representation
    """
    def __init__(self, latitude: float = 0., longitude: float = 0., altitude: float = 0.):
        """
        Construct
        :param latitude: the latitude
        :param longitude: the longitude
        :param altitude: the altitude
        """
        self.latitude = latitude
        self.longitude = longitude
        self.altitude = altitude

    def __str__(self) -> str:
        return "Pos long:'{}'-lat:'{}'-alt:'{}'".format(self.longitude,self.latitude,self.altitude)


class GPS_Sensor:
    """
    The abstract representation of the GPS sensor
    """
    def __init__(self, role_name: str):
        """
        Constructor
        :param role_name: the role name of the vehicle
        """
        self.position = GPS_Position()

        self.subscriber = rospy.Subscriber("/carla/{}/gnss/gnss1/fix".format(role_name), NavSatFix, self.__update_position)

        self.listener = None

    def __update_position(self, navSatFixMessage: NavSatFix):
        """
        Internal method to update the position data
        :param navSatFixMessage: the message
        :return: None
        """
        self.position = GPS_Position(navSatFixMessage.latitude, navSatFixMessage.longitude, navSatFixMessage.altitude)
        if self.listener != None:
            self.listener(self.position)

    def get_position(self) -> GPS_Position:
        """
        Return the current position
        :return:the current GPS_Position
        """
        return self.position

    def set_on_position_listener(self,func):
        """
        Set function to be called with the GPS_Position as parameter
        :param func: the function
        :return: None
        """
        self.listener = func;


