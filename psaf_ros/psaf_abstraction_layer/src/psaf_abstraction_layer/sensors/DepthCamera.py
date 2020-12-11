import rospy
from sensor_msgs.msg import Image,CameraInfo
import numpy as np


class DepthCamera:
    MAX_METERS = 1000

    def __init__(self, role_name: str = "ego_vehicle"):

        # 2d image with distance in meters max 1000
        self.image = None
        self.__subscriber = rospy.Subscriber("/carla/{}/camera/depth/front/image_depth".format(role_name), Image,
                                             self.__update_image)

        self.__listener = None

    def __update_image(self, image_msg: Image):
        """
        Internal method to update the position data
        :param image_msg: the message
        :return: None
        """
        array = np.frombuffer(image_msg.data, dtype=np.dtype("uint8"))
        array = np.reshape(array, (image_msg.height, image_msg.width,4))
        array = array.astype(np.float32)
        # Apply (R + G * 256 + B * 256 * 256) / (256 * 256 * 256 - 1).

        self.image = np.dot(array[:, :, :3], [65536.0, 256.0, 1.0])
        self.image /= (16777215.0/self.MAX_METERS)  # (256.0 * 256.0 * 256.0 - 1.0)

        if self.__listener != None:
            self.__listener(self.image)

    def get_image(self):
        """
        Return the current depth image
        :return:the current image
        """
        return self.position

    def set_on_image_listener(self, func):
        """
        Set function to be called with the depth image as parameter
        :param func: the function
        :return: None
        """
        self.__listener = func;
