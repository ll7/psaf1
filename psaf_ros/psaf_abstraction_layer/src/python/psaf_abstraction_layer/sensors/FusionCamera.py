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


def get_topic(role_name: str = "ego_vehicle") -> str:
    return f"/psaf/sensors/{role_name}/fusionCamera"


class FusionCameraService:

    def __init__(self, role_name: str = "ego_vehicle", time_threshold=0.1):
        super().__init__()

        # Threshold between two image in seconds -> smaller is better but makes it harder to find partners
        self.time_threshold = time_threshold

        self.__topic = get_topic(role_name)
        self.publisher = rospy.Publisher(self.topic, CombinedCameraImage, queue_size=4)
        # Init cameras
        id = "front"
        self.__depth_subscriber = rospy.Subscriber(f"/carla/{role_name}/camera/depth/{id}/image_depth", Image,
                                                   self.__on_depth_image, queue_size=10)
        self.depth_images: Dict[Time, Image] = {}

        self.__rgb__subscriber = rospy.Subscriber(f"/carla/{role_name}/camera/rgb/{id}/image_color", Image,
                                                  self.__on_rgb_image_update, queue_size=10)
        self.rgb_images: Dict[Time, Image] = {}

        self.__segmentation_subscriber = rospy.Subscriber(
            f"/carla/{role_name}/camera/semantic_segmentation/{id}/image_segmentation", Image,
            self.__on_segment_image_update, queue_size=10)
        self.segmentation_images: Dict[Time, Image] = {}

        rospy.Timer(rospy.Duration(0.2),self.__timer_callback)
        # Lock to prevent data races while processing the image data
        self.processing_lock = Lock()

    @property
    def topic(self):
        return self.__topic

    def __on_depth_image(self, image_msg: Image):
        self.depth_images[image_msg.header.stamp] = image_msg

    def __on_rgb_image_update(self, image_msg: Image):
        self.rgb_images[image_msg.header.stamp] = image_msg

    def __on_segment_image_update(self, image_msg: Image):
        self.segmentation_images[image_msg.header.stamp] = image_msg

    @classmethod
    def __time_difference(cls, time_a, time_b) -> float:
        """
        Calculate the difference of to time stamps in seconds.
        :param time_a: time stamp A
        :param time_b: time stamp B
        :return: the difference in seconds
        """
        if time_a is None or time_b is None:
            return math.nan
        return float(time_a.secs - time_b.secs + (time_a.nsecs - time_b.nsecs) * (10 ** -9))

    @classmethod
    def __find_best_matches(cls, list_a: List[Time], list_b: List[Time]) -> Dict[Time, Time]:
        def match(bigger_list: List[Time], smaller_list: List[Time]) -> Dict[Time, Time]:

            bigger_sorted_list = bigger_list #sorted(bigger_list, key=lambda x: x.nsecs * 10 ** -9 + x.secs)
            smaller_sorted_list = smaller_list #sorted(smaller_list, key=lambda x: x.nsecs * 10 ** -9 + x.secs)

            lower_bound = 0
            upper_bound = len(bigger_sorted_list)
            # dictionary that will contain the matches
            result = {}
            # Beginning of the algorithm
            for reference_value in smaller_sorted_list:
                # The last value of the bigger_sorted_list that might match to the current lessFreqValue
                possible_match = None
                # counter for the checked values in ine moreDataSorted list
                counter = 0
                for i in range(lower_bound, upper_bound):
                    # The value of the list that contains more elements -> higher measuring frequency
                    more_freq_value = bigger_sorted_list[i]
                    if possible_match is not None \
                            and abs(cls.__time_difference(reference_value, possible_match)) < abs(
                        cls.__time_difference(reference_value, more_freq_value)):
                        # If there is already possible match and if the difference between the current timestamp
                        # and the reference timestamp is greater than difference between timestamp of the last
                        # possible match and the  reference timestamp, all upcoming elements will have a greater
                        # difference and won't be an appropriate match
                        break

                    # Store the possible match
                    possible_match = more_freq_value
                    # Increment counter
                    counter += 1
                    if reference_value <= possible_match:
                        # if the reference time is older than the value of possible match all new values of the
                        # sorted(!) list will have a bigger difference
                        break

                if possible_match is not None:  # if there is a possible match
                    result[reference_value] = possible_match  # store the best match inside the result list
                    # increase the lower bound because all value with smaller timestamp that the match won't be suitable
                    # for future matches
                    lower_bound += counter

            return result

        # End helper function

        if len(list_a) >= len(list_b):
            return match(list_a, list_b)
        else:
            return dict(map(lambda x: (x[1], x[0]), match(list_b, list_a).items()))

    def __timer_callback(self,event):
        self.__match_images()

    def __match_images(self):


        with self.processing_lock:
            # Get the time list and use a copy of them to capture only the current situation that might change during
            # the execution
            segmentation_times = list(self.segmentation_images.keys())
            rgb_times = list(self.rgb_images.keys())
            depth_times = list(self.depth_images.keys())

            # Find the matches for both sensors
            rgb_matches = self.__find_best_matches(segmentation_times, rgb_times)
            depth_matches = self.__find_best_matches(segmentation_times, depth_times)

            # Now filter for the matches that meet the time threshold
            filtered_rgb_matches = dict(filter(
                lambda match_pair: abs(self.__time_difference(match_pair[0], match_pair[1])) <= self.time_threshold,
                rgb_matches.items()))
            filtered_depth_matches = dict(filter(
                lambda match_pair: abs(self.__time_difference(match_pair[0], match_pair[1])) <= self.time_threshold,
                depth_matches.items()))

            # stop here if we couldn't find any matches that meet our appropriate conditions and wishes
            if len(filtered_rgb_matches) == 0 or len(filtered_depth_matches) == 0:
                return

            # Combine both match list to find the matches that are matched with the same segmentation image
            # step 1: get the intersection of the key sets
            key_intersection = sorted(filtered_rgb_matches.keys() & filtered_depth_matches.keys())

            # stop here if we couldn't find any matches that meet our appropriate conditions and wishes
            if len(key_intersection) == 0:
                return
            # step 2: Retrieve the image data
            result: Dict[Time, Tuple[np.ndarray, np.ndarray, np.ndarray]] = {}
            # fill dictionary
            for key in key_intersection:
                try:
                    result[key] = (
                        self.segmentation_images[key], self.rgb_images[filtered_rgb_matches[key]],
                        self.depth_images[filtered_depth_matches[key]])
                except KeyError:
                    pass
            # step 3: remove all images that are older than the newest key
            time_border = key_intersection[-1]  # get the time stamp that is matched to the newest segmentation image
            for e in filter(lambda x: x <= time_border, segmentation_times):
                del self.segmentation_images[e]
            # get the time stamp that is matched to the newest segmentation image
            time_border = filtered_rgb_matches[key_intersection[-1]]
            for e in filter(lambda x: x <= time_border, rgb_times):
                del self.rgb_images[e]
            # get the time stamp that is matched to the newest segmentation image
            time_border = filtered_depth_matches[key_intersection[-1]]
            for e in filter(lambda x: x <= time_border, depth_times):
                del self.depth_images[e]

            for time, (segmentation_image, rgb_image, depth_image) in result.items():
                # Filter segmentation camera image for the given tags
                self.__publish(segmentation_image, rgb_image, depth_image)

            # Tidy up
            del segmentation_times, rgb_times, depth_times, filtered_rgb_matches, filtered_depth_matches

    def __publish(self, seg: Image, rgb: Image, depth: Image):
        msg = CombinedCameraImage()
        msg.segmentation = seg
        msg.rgb = rgb
        msg.depth = depth
        self.publisher.publish(msg)


class FusionCamera:
    """
    Abstraction layer for a fusion camera
    """

    def __init__(self, role_name: str = "ego_vehicle", visible_tags: Set[SegmentationTag] = None,queue_size = 1 ):
        # 2d image with distance in meters max 1000

        self.segmentation_image = None
        self.rgb_image = None
        self.depth_image = None

        self.visible_tags = visible_tags

        self.__subscriber = rospy.Subscriber(get_topic(role_name), CombinedCameraImage,
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
        print(f"Camera Age {age.to_sec()}s")
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

    # s = FusionCameraService(time_threshold=0.2)
    sensor = FusionCamera(queue_size=1)
    sensor.set_on_image_listener(store_image)
    rospy.spin()
