import cv2
import rospy
import time
import numpy as np
import os

from docutils.nodes import image

from psaf_abstraction_layer.DepthCamera import DepthCamera
from psaf_abstraction_layer.RGBCamera import RGBCamera
import torch
from torchvision.transforms import transforms

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
        print("[INFO] init device")
        self.device = torch.device("cuda:0" if torch.cuda.is_available() else "cpu")  # for using the GPU in pytorch
        print("[INFO] Device:" + str(self.device))
        # load our YOLO object detector trained on COCO dataset (3 classes)
        print("[INFO] loading basic YOLO from torch hub...")
        model = torch.hub.load('ultralytics/yolov5', 'yolov5m', classes=3)
        print("[INFO] loading YOLO from disk...")
        ckpt = torch.load('../../models/yolov5m.pt')['model']  # load checkpoint
        model.load_state_dict(ckpt.state_dict())  # load state_dict
        model.names = ckpt.names  # define class names
        self.labels = ckpt.names # copy labels
        model.to(self.device)
        self.net = model.autoshape() # Handles arleady to device, but must be called after to device
        self.net.eval()
        torch.no_grad() # reduce memory consumption and improve speed

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

        input = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        layerOutputs = self.net.forward(input)
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
                if score > self.confidence_min:
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
