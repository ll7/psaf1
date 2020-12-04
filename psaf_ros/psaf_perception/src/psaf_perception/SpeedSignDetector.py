import cv2
import rospy
import time
import numpy as np
import os
from psaf_abstraction_layer.DepthCamera import DepthCamera
from psaf_abstraction_layer.RGBCamera import RGBCamera


class DetectedObject:
    """
    Data class for detected elements
    """

    def __init__(self, x: int = 0, y: int = 0, w: int = 0, h: int = 0, label: str = "UNKNOWN",
                 confidence: float = 0.01):
        """

        :param x: x coord in image
        :param y: y coord in image
        :param h: height of bounding box
        :param w: weight of bounding box
        :param label: the class label name
        :param confidence: the confidence value
        """
        self.x = x
        self.y = y
        self.h = h
        self.w = w
        self.label = label
        self.confidence = confidence

        self.__listener = None


class SpeedSignDetector:
    def __init__(self, role_name: str = "ego_vehicle"):

        self.confidence_min = 0.5
        self.threshold = 0.7

        self.labels = ("speed_30", "speed_60", "speed_90")

        # derive the paths to the YOLO weights and model configuration
        weightsPath = os.path.abspath("../../models/yolov3-tiny-obj_5000.weights")
        configPath = os.path.abspath("../../models/yolov3-tiny.cfg")
        # load our YOLO object detector trained on COCO dataset (80 classes)W
        print("[INFO] loading YOLO from disk...")
        self.net = cv2.dnn.readNetFromDarknet(configPath, weightsPath)

        # determine only the *output* layer names that we need from YOLO
        ln = self.net.getLayerNames()
        self.ln = [ln[i[0] - 1] for i in self.net.getUnconnectedOutLayers()]

        self.depth_camera = DepthCamera(role_name)
        # self.depth_camera.set_on_image_listener(self.__on_depth_image)
        self.rgb_camera = RGBCamera(role_name)
        self.rgb_camera.set_on_image_listener(self.__on_rgb_image_update)
        self.depth_image = None

    def __on_depth_image(self, image):
        scale_percent = 600 / image.shape[0]  # percent of original size
        width = int(image.shape[1] * scale_percent)
        height = int(image.shape[0] * scale_percent)
        dim = (width, height)
        # resize image
        image = cv2.resize(image, dim, interpolation=cv2.INTER_AREA)
        over_width = (width - 800.0)
        min_x = int(over_width / 2)
        max_x = int(width - over_width / 2)
        self.depth_image = image[:, min_x:max_x]

    def __on_rgb_image_update(self, image):
        (H, W) = image.shape[:2]

        blob = cv2.dnn.blobFromImage(image, 1 / 255.0, (416, 416),
                                     swapRB=True, crop=False)
        self.net.setInput(blob)
        layerOutputs = self.net.forward(self.ln)
        # initialize our lists of detected bounding boxes, confidences, and
        # class IDs, respectively
        boxes = []
        confidences = []
        classIDs = []
        # loop over each of the layer outputs
        for output in layerOutputs:
            # loop over each of the detections
            for detection in output:
                # extract the class ID and confidence (i.e., probability) of
                # the current object detection
                scores = detection[5:]
                classID = np.argmax(scores)
                confidence = scores[classID]
                # filter out weak predictions by ensuring the detected
                # probability is greater than the minimum probability
                if confidence > self.confidence_min:
                    # scale the bounding box coordinates back relative to the
                    # size of the image, keeping in mind that YOLO actually
                    # returns the center (x, y)-coordinates of the bounding
                    # box followed by the boxes' width and height
                    box = detection[0:4] * np.array([W, H, W, H])
                    (centerX, centerY, width, height) = box.astype("int")
                    # use the center (x, y)-coordinates to derive the top and
                    # and left corner of the bounding box
                    x = int(centerX - (width / 2))
                    y = int(centerY - (height / 2))
                    # update our list of bounding box coordinates, confidences,
                    # and class IDs
                    boxes.append([x, y, int(width), int(height)])
                    confidences.append(float(confidence))
                    classIDs.append(classID)

        # List oif detected elements
        detected = []
        # apply non-maxima suppression to suppress weak, overlapping bounding
        # boxes
        idxs = cv2.dnn.NMSBoxes(boxes, confidences, confidence, self.threshold)

        # ensure at least one detection exists
        if len(idxs) > 0:
            # loop over the indexes we are keeping
            for i in idxs.flatten():
                detected.append(
                    DetectedObject(boxes[i][0], boxes[i][1], boxes[i][2], boxes[i][3], self.labels[classIDs[i]],
                                   confidences[0]))

            # inform listener
            if self.__listener != None:
                self.__listener(detected)

    def set_on_detection_listener(self, func):
        """
        Set function to be called with detected bounding boxes
        :param func: the function
        :return: None
        """
        self.__listener = func;


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
                color = (0, 255, 0)
                cv2.rectangle(image, (x, y), (x + w, y + h), color, 2)
                text = "{}: {:.4f}".format(element.label, element.confidence)
                cv2.putText(image, text, (x, y - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)

        # show the output image
        cv2.imshow("RGB", image)
        cv2.waitKey(1)


    def on_detected(detected_list):
        global detected_r
        detected_r = detected_list


    cam = RGBCamera()

    cam.set_on_image_listener(store_image)

    s = SpeedSignDetector()
    s.set_on_detection_listener(on_detected)
    rospy.spin()
