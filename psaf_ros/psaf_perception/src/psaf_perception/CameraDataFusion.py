import math
from threading import Lock
from typing import Set, Tuple, List, Callable, Dict

import cv2
import numpy as np
import rospy
from genpy import Time

from psaf_abstraction_layer.sensors.DepthCamera import DepthCamera
from psaf_abstraction_layer.sensors.RGBCamera import RGBCamera
from psaf_abstraction_layer.sensors.SegmentationCamera import SegmentationCamera, Tag as SegmentationTag


class CameraDataFusion:

    def __init__(self, role_name: str = "ego_vehicle", time_threshold=0.1, visible_tags: Set[SegmentationTag] = None):
        super().__init__()

        self.visible_tags = visible_tags

        # Threshold between two image in seconds -> smaller is better but makes it harder to find partners
        self.time_threshold = time_threshold

        # Init camera
        self.depth_camera = DepthCamera(role_name, "front")
        self.depth_camera.set_on_image_listener(self.__on_depth_image)
        self.depth_images: Dict[Time, np.ndarray] = {}

        self.rgb_camera = RGBCamera(role_name, "front")
        self.rgb_camera.set_on_image_listener(self.__on_rgb_image_update)
        self.rgb_images: Dict[Time, np.ndarray] = {}

        self.segmentation_camera = SegmentationCamera(role_name, "front")
        self.segmentation_camera.set_on_image_listener(self.__on_segment_image_update)
        self.segmentation_images: Dict[Time, np.ndarray] = {}

        self.__listener = None

        # Lock to prevent data races while processing the image data
        self.processing_lock = Lock()

    def __on_depth_image(self, image, time):
        self.depth_images[time] = image

    def __on_rgb_image_update(self, image, time):
        self.rgb_images[time] = image

    def __on_segment_image_update(self, image, time):
        self.segmentation_images[time] = image
        self.__match_images(self.segmentation_images, self.rgb_images,
                            self.depth_images)

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

            bigger_sorted_list = sorted(bigger_list, key=lambda x: x.nsecs * 10 ** -9 + x.secs)
            smaller_sorted_list = sorted(smaller_list, key=lambda x: x.nsecs * 10 ** -9 + x.secs)

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

    def __match_images(self, segmentation_images: Dict[Time, np.ndarray], rgb_images: Dict[Time, np.ndarray],
                       depth_images: Dict[Time, np.ndarray]):

        # Get the time list and use a copy of them to capture only the current situation that might change during
        # the execution
        segmentation_times = list(segmentation_images.keys())
        rgb_times = list(rgb_images.keys())
        depth_times = list(depth_images.keys())

        with self.processing_lock:
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
                        segmentation_images[key], rgb_images[filtered_rgb_matches[key]],
                        depth_images[filtered_depth_matches[key]])
                except KeyError:
                    pass
            # step 3: remove all images that are older than the newest key
            time_border = key_intersection[-1]  # get the time stamp that is matched to the newest segmentation image
            for e in filter(lambda x: x <= time_border, segmentation_times):
                del segmentation_images[e]
            # get the time stamp that is matched to the newest segmentation image
            time_border = filtered_rgb_matches[key_intersection[-1]]
            for e in filter(lambda x: x <= time_border, rgb_times):
                del rgb_images[e]
            # get the time stamp that is matched to the newest segmentation image
            time_border = filtered_depth_matches[key_intersection[-1]]
            for e in filter(lambda x: x <= time_border, depth_times):
                del depth_images[e]

            for time, (segmentation_image, rgb_image, depth_image) in result.items():
                # Filter segmentation camera image for the given tags
                if self.visible_tags is not None:
                    segmentation_image = SegmentationCamera.filter_for_tags(segmentation_image, self.visible_tags)

                # Call listener method
                if self.__listener is not None:
                    self.__listener(segmentation_image, rgb_image, depth_image, time)

            # Tidy up
            del segmentation_times, rgb_times, depth_times, filtered_rgb_matches, filtered_depth_matches

    def set_on_image_data_listener(self, func: Callable[[np.ndarray, np.ndarray, np.ndarray, Time], None]):
        """
        Set function to be called with image data
        :param func: the function ( segmentation_image, rgb_image, depth_image, time) -> None
        :return: None
        """
        self.__listener = func


class CameraDataFusionWrapper:
    # TODO Bug listener of second wrapper with same instance seems not to be called
    """
    Wrapper for CameraDataFusion to reduce the computation time for identical camera data fusions by
    applying a singleton wrapper patter
    """

    instances = {}
    wrapper_listeners = {}

    def __init__(self, role_name: str = "ego_vehicle", time_threshold=0.1, visible_tags: Set[SegmentationTag] = None):
        self.instance = CameraDataFusionWrapper.__get_instance(role_name, time_threshold)
        self.visible_tags = visible_tags

        self.__listener = None

        CameraDataFusionWrapper.wrapper_listeners[self.instance].append(self.__on_new_data)

    @classmethod
    def __get_instance(cls, role_name: str = "ego_vehicle", time_threshold=0.1):
        """
        Internal helper method to crate the instances if necessary
        :param role_name:
        :param time_threshold:
        :return:
        """
        parameters = (role_name, time_threshold)
        if parameters in cls.instances.keys():
            return cls.instances[parameters]
        else:
            instance = CameraDataFusion(role_name=role_name, time_threshold=time_threshold, visible_tags=None)
            instance.set_on_image_data_listener(
                lambda segmentation_image, rgb_image, depth_image, time:
                cls.__listener_wrapper(instance, segmentation_image, rgb_image, depth_image, time))
            cls.wrapper_listeners[instance] = []
            cls.instances[parameters] = instance
            return instance

    @classmethod
    def __listener_wrapper(cls, instance, segmentation_image, rgb_image, depth_image, time):
        for each in cls.wrapper_listeners[instance]:
            # Process(target=each,args = [ segmentation_image,rgb_image, depth_image, time]).start()
            each(segmentation_image, rgb_image, depth_image, time)

    def __on_new_data(self, segmentation_image, rgb_image, depth_image, time):
        # Filter segmentation camera image for the given tags
        if self.visible_tags is not None:
            segmentation_image = SegmentationCamera.filter_for_tags(segmentation_image, self.visible_tags)

        # Call listener method
        if self.__listener is not None:
            self.__listener(segmentation_image, rgb_image, depth_image, time)

    def set_on_image_data_listener(self, func: Callable[[np.ndarray, np.ndarray, np.ndarray, Time], None]):
        """
        Set function to be called with image data
        :param func: the function ( segmentation_image, rgb_image, depth_image, time) -> None
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
    rospy.init_node("DetectionTest")


    def store_image(seg_image, rgb_image, depth_image, _):
        # Create one big image
        image = np.vstack((seg_image,
                           rgb_image,
                           cv2.cvtColor((depth_image / DepthCamera.MAX_METERS * 255).astype('uint8'),
                                        cv2.COLOR_GRAY2BGR)))
        show_image("Fusion", image)


    s = CameraDataFusionWrapper()
    s.set_on_image_data_listener(store_image)
    rospy.spin()
