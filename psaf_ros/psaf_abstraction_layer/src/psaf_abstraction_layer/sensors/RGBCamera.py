import rospy
from sensor_msgs.msg import Image
from cv_bridge.core import CvBridge


class RGBCamera:

    def __init__(self,role_name:str= "ego_vehicle"):

        self.image = None
        self.bridge = CvBridge()
        self.__subscriber = rospy.Subscriber("/carla/{}/camera/rgb/front/image_color".format(role_name), Image,
                                             self.__update_image)

        self.__listener = None

    def __update_image(self, image_msg:Image):
        """
        Internal method to update the position data
        :param image_msg: the message
        :return: None
        """
        self.image = self.bridge.imgmsg_to_cv2(image_msg,desired_encoding='bgr8')

        if self.__listener != None:
            self.__listener(self.image)

    def get_image(self):
        """
        Return the current rgb image
        :return:the current image
        """
        return self.position

    def set_on_image_listener(self, func):
        """
        Set function to be called with the rgb image as parameter
        :param func: the function
        :return: None
        """
        self.__listener = func;

