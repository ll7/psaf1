import cv2
import rospy
from sensor_msgs.msg import Image
import numpy as np


class DepthCamera:
    """
    Abstraction layer for a depth camera
    It converts the distance values to meters and stores them in an numpy array
    """

    MAX_METERS = 1000  # maximum view distance

    def __init__(self, role_name: str = "ego_vehicle", id: str = "front"):
        # 2d image with distance in meters max 1000
        self.image = None
        self.__subscriber = rospy.Subscriber(f"/carla/{role_name}/camera/depth/{id}/image_depth", Image,
                                             self.__update_image)

        self.__listener = None

    def __update_image(self, image_msg: Image):
        """
        Internal method to update the distance data
        :param image_msg: the message
        :return: None
        """
        array = np.frombuffer(image_msg.data, dtype=np.dtype("uint8"))
        array = np.reshape(array, (image_msg.height, image_msg.width, 4))
        array = array.astype(np.float32)
        # Apply (R + G * 256 + B * 256 * 256) / (256 * 256 * 256 - 1).

        normed= np.dot(array[:, :, :3], [65536.0, 256.0, 1.0]) / 16777215.0
        self.image = normed * self.MAX_METERS  # 65536.0 = (256.0 * 256.0 * 256.0 - 1.0)

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
        """        # if self.__listener != None:
        #     self.__listener(self.image)
        self.__listener = func



# Show case code
if __name__ == "__main__":
    rospy.init_node("DepthCameraTest")

    def show_image(image):

        show = (image/DepthCamera.MAX_METERS * 255).astype('uint8')
        # show the output image
        cv2.imshow("Depth", cv2.cvtColor(show, cv2.COLOR_GRAY2BGR))
        cv2.waitKey(1)



    cam = DepthCamera()

    cam.set_on_image_listener(show_image)
    rospy.spin()
