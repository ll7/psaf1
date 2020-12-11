import cv2
import numpy as np
import rospy
import torch

from psaf_abstraction_layer.DepthCamera import DepthCamera
from psaf_abstraction_layer.RGBCamera import RGBCamera
from psaf_perception.AbstractDetector import DetectedObject, AbstractDetector


class SpeedSignDetector(AbstractDetector):
    def __init__(self, role_name: str = "ego_vehicle"):
        super().__init__()

        self.confidence_min = 0.70
        self.threshold = 0.7
        print("[INFO] init device")
        self.device = torch.device("cuda:0" if torch.cuda.is_available() else "cpu")  # for using the GPU in pytorch
        print("[INFO] Device:" + str(self.device))
        # load our YOLO object detector trained on COCO dataset (3 classes)
        print("[INFO] loading basic YOLO from torch hub...")
        model = torch.hub.load('ultralytics/yolov5', 'yolov5m', classes=3)
        print("[INFO] loading YOLO from disk...")
        ckpt = torch.load('../../models/yolov5m-e250-frozen_backbone/weights/best.pt')['model']  # load checkpoint
        model.load_state_dict(ckpt.state_dict())  # load state_dict
        model.names = ckpt.names  # define class names
        self.labels = ckpt.names  # copy labels
        model.to(self.device)
        # Autoshape wraps model and send data already to device, but must be called after model to device
        self.net = model.autoshape()
        self.net.eval()
        torch.no_grad()  # reduce memory consumption and improve speed

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
                    detected.append(
                        DetectedObject(boxes[i][0], boxes[i][1], boxes[i][2], boxes[i][3], self.labels[classIDs[i]],
                                       confidences[i]))

        self.inform_listener(detected)


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
                color = (0, 255, 0)
                cv2.rectangle(image, (x, y), (x + w, y + h), color, 2)
                text = "{}: {:.4f}".format(element.label, element.confidence)
                cv2.putText(image, text, (x - 5, y - 5), cv2.FONT_HERSHEY_SIMPLEX, 1, color, 2)

        image = cv2.resize(image, (1000, 300), interpolation=cv2.INTER_AREA)
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
