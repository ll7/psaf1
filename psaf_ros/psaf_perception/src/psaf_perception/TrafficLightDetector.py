import cv2
import numpy as np
import rospy
import torch

from psaf_abstraction_layer.sensors.RGBCamera import RGBCamera
from psaf_perception.AbstractDetector import DetectedObject, AbstractDetector, Labels


class TrafficLightDetector(AbstractDetector):
    """
    Detector for traffic lights
    """

    def __init__(self, role_name: str = "ego_vehicle"):
        super().__init__()

        self.confidence_min = 0.65
        self.threshold = 0.7
        print("[INFO] init device")
        self.device = torch.device("cuda:0" if torch.cuda.is_available() else "cpu")  # for using the GPU in pytorch
        print("[INFO] Device:" + str(self.device))
        # load our YOLO object detector trained on COCO dataset (3 classes)
        print("[INFO] loading basic YOLO from torch hub...")
        model = torch.hub.load('ultralytics/yolov5', 'yolov5s', pretrained=True)
        print("[INFO] loading YOLO from disk...")
        model.to(self.device)
        # Autoshape wraps model and send data already to device, but must be called after model to device
        self.net = model.autoshape()
        self.net.eval()
        torch.no_grad()  # reduce memory consumption and improve speed

        self.rgb_camera = RGBCamera(role_name)
        self.rgb_camera.set_on_image_listener(self.__on_rgb_image_update)

    def __extract_label(self, image) -> Labels:
        """
        Analyze the given image of the traffic light and returns the corresponding label
        :param image: the important part of the camera image
        :return: the Label
        """

        return TrafficLightClassifier.classify(image)

    def __on_rgb_image_update(self, image):
        (H, W) = image.shape[:2]

        layerOutputs = self.net.forward(image)
        # initialize our lists of detected bounding boxes, confidences, and
        # class IDs, respectively
        boxes = []
        confidences = []
        classIDs = []
        for output in layerOutputs.xywhn:
            # loop over each of the detections
            for detection in output.cpu().numpy():
                # extract the class ID and confidence (i.e., probability) of
                # the current object detection
                classID = int(detection[5])
                score = detection[4]
                # filter out weak predictions by ensuring the detected
                # probability is greater than the minimum probability
                if score > self.confidence_min and self.net.names[classID] == 'traffic light':
                    # scale the bounding box coordinates back relative to the
                    # size of the image, keeping in mind that YOLO actually
                    # returns the center (x, y)-coordinates of the bounding
                    # box followed by the boxes' width and height
                    # TODO keep relative coordinates
                    box = detection[0:4] * np.array([W, H, W, H])
                    (centerX, centerY, width, height) = box.astype("int")
                    # use the center (x, y)-coordinates to derive the top and
                    # and left corner of the bounding box
                    x = int(centerX - (width / 2))
                    y = int(centerY - (height / 2))
                    # update our list of bounding box coordinates, confidences,
                    # and class IDs
                    boxes.append([x, y, int(width), int(height)])
                    confidences.append(float(score))
                    classIDs.append(classID)

        # List oif detected elements
        detected = []
        # apply non-maxima suppression to suppress weak, overlapping bounding
        # boxes
        idxs = cv2.dnn.NMSBoxes(boxes, confidences, self.confidence_min, self.threshold)

        # ensure at least one detection exists
        if len(idxs) > 0:
            # loop over the indexes we are keeping
            for i in idxs.flatten():
                if boxes[i][2] * boxes[i][3] > (
                        450):
                    x1 = int(boxes[i][0])
                    x2 = boxes[i][0] + boxes[i][2]
                    y1 = int(boxes[i][1])
                    y2 = boxes[i][1] + boxes[i][3]
                    label = self.__extract_label(
                        image[int(y1):y2, x1:x2])
                    detected.append(
                        DetectedObject(boxes[i][0], boxes[i][1], boxes[i][2], boxes[i][3], label,
                                       confidences[i]))

        self.inform_listener(detected)


class TrafficLightClassifier:
    """
    Based on https://www.kaggle.com/photunix/classify-traffic-lights-with-pre-trained-cnn-model/comments,
    Released under the apache 2.0 licence
    """

    @classmethod
    def __high_saturation_region_mask(cls, hsv_image, s_thres=0.6):
        """
        Creates an numpy mask to filter for areas with a high saturation
        :param hsv_image: the image
        :param s_thres: the threshold [0;1]
        :return: the mask
        """
        if hsv_image.dtype == np.int:
            idx = (hsv_image[:, :, 1].astype(np.float) / 255.0) < s_thres
        else:
            idx = (hsv_image[:, :, 1].astype(np.float)) < s_thres
        mask = np.ones_like(hsv_image[:, :, 1])
        mask[idx] = 0
        return mask

    @classmethod
    def __channel_percentile(cls, single_chan_image, percentile):
        """
        Wrapper for np.percentile for single channel images
        :param single_chan_image: the single channel image
        :param percentile: the percentile
        :return: threshold value normed to 0-255
        """
        sq_image = np.squeeze(single_chan_image)
        assert len(sq_image.shape) < 3

        thres_value = np.percentile(sq_image.ravel(), percentile)

        return float(thres_value) / 255.0

    @classmethod
    def __high_value_region_mask(cls, hsv_image, v_thres=0.6):
        """
        Creates an numpy mask to filter for areas with a high value
        :param hsv_image: the hsv image
        :param v_thres: the threshold [0;1]
        :return: the numpy mask
        """
        if hsv_image.dtype == np.int:
            idx = (hsv_image[:, :, 2].astype(np.float) / 255.0) < v_thres
        else:
            idx = (hsv_image[:, :, 2].astype(np.float)) < v_thres
        mask = np.ones_like(hsv_image[:, :, 2])
        mask[idx] = 0
        return mask

    @classmethod
    def __get_masked_hue_values(cls, rgb_image):
        """
        Get the pixels in the RGB image that has high saturation (S) and value (V) in HSV chanels

        :param rgb_image: image (height, width, channel)
        :return: a 1-d array
        """

        hsv_test_image = cv2.cv2.cvtColor(rgb_image, cv2.COLOR_RGB2HSV)
        s_thres_val = cls.__channel_percentile(hsv_test_image[:, :, 1], percentile=80)
        v_thres_val = cls.__channel_percentile(hsv_test_image[:, :, 2], percentile=30)
        val_mask = cls.__high_value_region_mask(hsv_test_image, v_thres=v_thres_val)
        sat_mask = cls.__high_saturation_region_mask(hsv_test_image, s_thres=s_thres_val)
        masked_hue_image = hsv_test_image[:, :, 0] * 180
        # Note that the following statement is not equivalent to
        # masked_hue_1d= (maksed_hue_image*np.logical_and(val_mask,sat_mask)).ravel()
        # Because zero in hue channel means red, we cannot just set unused pixels to zero.
        masked_hue_1d = masked_hue_image[np.logical_and(val_mask, sat_mask)].ravel()

        return masked_hue_1d

    @classmethod
    def __convert_to_hue_angle(cls, hue_array):
        """
        Convert the hue values from [0,179] to radian degrees [-pi, pi]

        :param hue_array: array-like, the hue values in degree [0,179]
        :return: the angles of hue values in radians [-pi, pi]
        """

        hue_cos = np.cos(hue_array * np.pi / 90)
        hue_sine = np.sin(hue_array * np.pi / 90)

        hue_angle = np.arctan2(hue_sine, hue_cos)

        return hue_angle

    @classmethod
    def __get_rgy_color_mask(cls, hue_value):
        """
        return a tuple of np.ndarray that sets the pixels with red, green and yellow matrices to be true

        :param hue_value:
        :return:
        """

        red_index = np.logical_and(hue_value > -0.4, hue_value < 0.9)
        green_index = np.logical_and(hue_value > 1.2, hue_value < 2)
        yellow_index = np.logical_and(hue_value > 1.9, hue_value < 2.2)

        # red_index = np.logical_and(hue_value > (-0.125 * np.pi), hue_value < (0.125 * np.pi))
        # green_index = np.logical_and(hue_value > (0.66 * np.pi), hue_value < 1.0 * np.pi)
        # yellow_index = np.logical_and(hue_value > (0.25 * np.pi), hue_value < (0.42 * np.pi))

        return red_index, green_index, yellow_index

    @classmethod
    def __classify_color_by_range(cls, hue_value):
        """
        Determine the color (red, yellow or green) in a hue value array

        :param hue_value: hue_value is radians
        :return: the color index ['red', 'yellow', 'green', '_', 'unknown']
        """

        red_index, green_index, yellow_index = cls.__get_rgy_color_mask(hue_value)

        color_counts = np.array([np.sum(red_index) / len(hue_value),
                                 np.sum(yellow_index) / len(hue_value),
                                 np.sum(green_index) / len(hue_value)])

        return color_counts

    @classmethod
    def __classify_color_cropped_image(cls, rgb_image):
        """
        Full pipeline of classifying the traffic light color from the traffic light image

        :param rgb_image: the RGB image array (height,width, RGB channel)
        :return: the count for every image
        """

        hue_1d_deg = cls.__get_masked_hue_values(rgb_image)

        if len(hue_1d_deg) == 0:
            return Labels.TrafficLightUnknown

        hue_1d_rad = cls.__convert_to_hue_angle(hue_1d_deg)

        count = cls.__classify_color_by_range(hue_1d_rad)

        return count

    @classmethod
    def classify(cls, rgb_image):
        H, W = rgb_image.shape[:2]
        h = int(H / 3)
        x1 = int(max([W / 3, 1]))
        x2 = int(max([2 * W / 3, W]))

        top = rgb_image[0:h, x1:x2, :]
        center = rgb_image[h:h * 2, x1:x2, :]
        bottom = rgb_image[h * 2:H, x1:x2, :]

        top_values = cls.__classify_color_cropped_image(top)
        center_values = cls.__classify_color_cropped_image(center)
        bottom_values = cls.__classify_color_cropped_image(bottom)

        red = top_values[0] > bottom_values[2] and np.argmax(top_values) == 0
        yellow = np.argmax(center_values) == 1
        green = top_values[0] < bottom_values[2]

        if red:
            return Labels.TrafficLightRed
        if green:
            return Labels.TrafficLightGreen
        if yellow:
            return Labels.TrafficLightYellow

        return Labels.TrafficLightUnknown


# Show case code
if __name__ == "__main__":
    rospy.init_node("DetectionTest")

    detected_r = None


    def store_image(image):
        global detected_r

        if detected_r is not None:
            for element in detected_r:
                # extract the bounding box coordinates
                (x, y) = (element.x, element.y)
                (w, h) = (element.w, element.h)
                # draw a bounding box rectangle and label on the image
                colorMap = {Labels.TrafficLightRed: (255, 0, 0), Labels.TrafficLightGreen: (0, 255, 0),
                            Labels.TrafficLightYellow: (255, 255, 0), Labels.TrafficLightOff: (0, 0, 255),
                            Labels.TrafficLightUnknown: (0, 0, 0)}
                color = colorMap.get(element.label, (0, 0, 0))
                cv2.rectangle(image, (x, y), (x + w, y + h), color, 2)
                text = "{}: {:.4f}".format(element.label.label_text, element.confidence)
                cv2.putText(image, text, (x - 5, y - 5), cv2.FONT_HERSHEY_SIMPLEX, 1, color, 2)

        # resize image to limits
        limit = (1000, 1000)  # height, width
        fac = 1.0
        if image.shape[0] > limit[0]:
            fac = limit[0] / image.shape[0]
        elif image.shape[1] > limit[1]:
            fac = limit[1] / image.shape[1]
        image = cv2.resize(image, (int(image.shape[1] * fac), int(image.shape[0] * fac)))
        # show the output image
        cv2.imshow("RGB", cv2.cvtColor(image, cv2.COLOR_RGB2BGR))
        cv2.waitKey(1)


    def on_detected(detected_list):
        global detected_r
        detected_r = detected_list


    cam = RGBCamera()

    cam.set_on_image_listener(store_image)

    s = TrafficLightDetector()
    s.set_on_detection_listener(on_detected)
    rospy.spin()
