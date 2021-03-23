import math
from threading import Lock
from typing import Callable, Set
from typing import Tuple, List, Dict

import cv2
import numpy
import numpy as np
import rospy
from cv_bridge import CvBridge
from genpy import Time
from psaf_messages.msg import CombinedCameraImage
from sensor_msgs.msg import Image
from std_msgs.msg import Time
from psaf_abstraction_layer.sensors.SegmentationCamera import Tag as SegmentationTag, SegmentationCamera


class FusionCamera:
    """
    Abstraction layer for a fusion camera
    """

    def __init__(self, role_name: str = "ego_vehicle",camera_name: str = "front", visible_tags: Set[SegmentationTag] = None,queue_size = 1 ):
        # 2d image with distance in meters max 1000

        self.segmentation_image = None
        self.rgb_image = None
        self.depth_image = None

        self.visible_tags = visible_tags

        self.__subscriber = rospy.Subscriber(f"/psaf/sensors/{role_name}/fusionCamera/{camera_name}/fusion_image", CombinedCameraImage,
                                             self.__update_image,queue_size=queue_size)

        self.__listener = None
        self.bridge = CvBridge()

    def __update_image(self, image_msg: CombinedCameraImage):
        """
        Internal method to update the distance data
        :param image_msg: the message
        :return: None
        """

        age = rospy.Time.now() - image_msg.segmentation.header.stamp
        self.segmentation_image = self.bridge.imgmsg_to_cv2(image_msg.segmentation, desired_encoding='rgb8')

        if self.visible_tags is not None:
            self.segmentation_image = SegmentationCamera.filter_for_tags(self.segmentation_image, self.visible_tags)

        self.rgb_image = cv2.cvtColor(self.bridge.imgmsg_to_cv2(image_msg.rgb, desired_encoding='bgr8'),
                                      cv2.COLOR_BGR2RGB)
        self.depth_image = self.bridge.imgmsg_to_cv2(image_msg.depth, desired_encoding='passthrough')

        if self.__listener != None:
            self.__listener(image_msg.segmentation.header.stamp, self.segmentation_image, self.rgb_image,
                            self.depth_image)

    def get_image(self):
        """
        Return the current depth image
        :return:the current image
        """
        return self.position

    def set_on_image_listener(self, func: Callable[[Time, numpy.ndarray, numpy.ndarray, numpy.ndarray], None]):
        """
        Set function to be called with the time, segmentation, rgb and depth image as parameter
        :param func: the function
        :return: None
        """
        self.__listener = func


# Show case code
def show_image(title, image):
    max_width, max_height = 1200, 800

    limit = (max_height, max_width)
    fac = 1.0
    if image.shape[0] > limit[0]:
        fac = limit[0] / image.shape[0]
    elif image.shape[1] > limit[1]:
        fac = limit[1] / image.shape[1]
    image = cv2.resize(image, (int(image.shape[1] * fac), int(image.shape[0] * fac)))
    # show the output image
    cv2.imshow(title, cv2.cvtColor(image, cv2.COLOR_RGB2BGR))
    cv2.waitKey(1)


if __name__ == "__main__":
    rospy.init_node("FusionCameraService")


    def store_image(time_stamp, seg_image, rgb_image, depth_image):
        from psaf_abstraction_layer.sensors.DepthCamera import DepthCamera

        age = rospy.Time.now() - time_stamp
        print(f"Age {age.to_sec()}s")
        # Create one big image
        image = np.vstack((seg_image,
                           rgb_image,
                           cv2.cvtColor((depth_image / DepthCamera.MAX_METERS * 255).astype('uint8'),
                                        cv2.COLOR_GRAY2BGR)))
        show_image("Fusion", image)
        import time
        time.sleep(0.2)

    sensor = FusionCamera(queue_size=1)
    sensor.set_on_image_listener(store_image)
    rospy.spin()
