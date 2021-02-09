from typing import Callable

import cv2
import numpy
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import Time


class DepthCamera:
    """
    Abstraction layer for a depth camera
    It converts the distance values to meters and stores them in an numpy array
    """

    MAX_METERS = 1000  # maximum view distance

    def __init__(self, role_name: str = "ego_vehicle", id: str = "front",queue_size=None):
        # 2d image with distance in meters max 1000
        self.image = None
        self.__subscriber = rospy.Subscriber(f"/carla/{role_name}/camera/depth/{id}/image_depth", Image,
                                             self.__update_image,queue_size=queue_size)

        self.__listener = None
        self.bridge = CvBridge()


    def __update_image(self, image_msg: Image):
        """
        Internal method to update the distance data
        :param image_msg: the message
        :return: None
        """

        self.image = self.bridge.imgmsg_to_cv2(image_msg,desired_encoding='passthrough')

        if self.__listener != None:
            self.__listener(self.image,image_msg.header.stamp)

    def get_image(self):
        """
        Return the current depth image
        :return:the current image
        """
        return self.image

    def set_on_image_listener(self, func:Callable[[numpy.ndarray,Time],None]):
        """
        Set function to be called with the depth image as parameter
        :param func: the function
        :return: None
        """
        self.__listener = func



# Show case code
if __name__ == "__main__":
    rospy.init_node("DepthCameraTest")

    def show_image(image,_):

        show = (image/DepthCamera.MAX_METERS * 255).astype('uint8')
        # show the output image
        cv2.imshow("Depth", cv2.cvtColor(show, cv2.COLOR_GRAY2BGR))
        cv2.waitKey(1)



    cam = DepthCamera()

    cam.set_on_image_listener(show_image)
    rospy.spin()
