import json
import os
from datetime import datetime
from typing import Tuple

import cv2
import numpy as np
import rospkg
import rospy
import torch
from torch.autograd import Variable
from torchvision.transforms import transforms

from psaf_abstraction_layer.sensors.RGBCamera import RGBCamera
from psaf_perception.CameraDataFusion import SegmentationTag, CameraDataFusion
from psaf_perception.detectors.AbstractDetector import Labels, AbstractDetector, DetectedObject


class TrafficSignDetector(AbstractDetector):

    def __init__(self, role_name: str = "ego_vehicle", use_gpu: bool = True):
        """
        Init the traffic sign detector
        :param role_name: the name of the vehicle to access the cameras
        :param use_gpu: whether the classification model should be loaded to the gpu
        """
        super().__init__()
        self.logger_name = "TrafficSignDetector"

        rospack = rospkg.RosPack()
        root_path = rospack.get_path('psaf_perception')

        self.confidence_min = 0.85
        self.threshold = 0.7
        self.canny_threshold = 100

        self.data_collect_path = None  # "/home/psaf1/Documents/traffic_sign_data"

        rospy.loginfo(f"init device (use gpu={use_gpu})", logger_name=self.logger_name)
        # select the gpu if allowed and a gpu is available
        self.device = torch.device("cuda:0" if use_gpu and torch.cuda.is_available() else "cpu")
        rospy.loginfo("Device:" + str(self.device), logger_name=self.logger_name)
        # load our model
        model_name = 'traffic_sign-classifier-2021-01-19-09:53:51'
        rospy.loginfo("loading classifier model from disk...", logger_name=self.logger_name)
        model = torch.load(os.path.join(root_path, f"models/{model_name}.pt"))

        with open(os.path.join(root_path, f"models/{model_name}.names")) as f:
            class_names = json.load(f)
        self.labels = {class_names['stop']: Labels.StopSign,
                       class_names['speed_30']: Labels.Speed30,
                       class_names['speed_60']: Labels.Speed60,
                       class_names['speed_90']: Labels.Speed90,
                       class_names['speed_limit_30']: Labels.SpeedLimit30,
                       class_names['speed_limit_40']: Labels.SpeedLimit40,
                       class_names['speed_limit_60']: Labels.SpeedLimit60,
                       }
        self.transforms = transforms.Compose([
            transforms.ToPILImage(),
            transforms.Resize((224, 224)),
            transforms.ToTensor(),
            transforms.Normalize([0.485, 0.456, 0.406], [0.229, 0.224, 0.225])
        ])
        model.to(self.device)
        self.net = model
        self.net.eval()
        torch.no_grad()  # reduce memory consumption and improve speed

        # init image source = combination of segmentation, rgb and depth camera
        self.combinedCamera = CameraDataFusion(role_name=role_name, time_threshold=0.08,
                                               visible_tags=set([SegmentationTag.TrafficSign]))
        self.combinedCamera.set_on_image_data_listener(self.__on_new_image_data)

    def __classify(self, image) -> Tuple[Labels, float]:
        """
        Classifies the given image
        :param image: the rgb image
        :return: the Label and the confidence as a tuple
        """

        image = self.transforms(image).unsqueeze(dim=0)
        imgblob = Variable(image).to(self.device)
        pred = torch.nn.functional.softmax(self.net(imgblob).cpu(), dim=1).detach().numpy()[0, :]

        hit = np.argmax(pred)
        label = self.labels.get(hit, None)

        return label, float(pred[hit])

    def __on_new_image_data(self, segmentation_image, rgb_image, depth_image, time):
        """
        Handles the new image data from the cameras
        :param segmentation_image: the segmentation image
        :param rgb_image: the rgb image
        :param depth_image: the depth data
        :param time: time stamp when the images where taken
        :return: none
        """
        (H, W) = segmentation_image.shape[:2]

        # List oif detected elements
        detected = []

        # Detect edges
        canny_output = cv2.Canny(segmentation_image, self.canny_threshold, self.canny_threshold * 2)
        contours, _ = cv2.findContours(canny_output, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        boxes = []
        classes = []
        confidences = []
        for i, c in enumerate(contours):
            contours_poly = cv2.approxPolyDP(c, 3, True)
            x1, y1, w, h = cv2.boundingRect(contours_poly)
            boxes.append(np.array([x1, y1, w, h]) / np.array([W, H, W, H]))
            x2 = x1 + w
            y2 = y1 + h

            mask = segmentation_image[y1:y2, x1:x2] == SegmentationTag.TrafficSign.color
            # get cropped rgb image
            crop_rgb = rgb_image[y1:y2, x1:x2, :]
            # use inverted mask to clear the background
            crop_rgb[np.logical_not(mask)] = 255
            classification, confidence = self.__classify(crop_rgb)
            classes.append(classification)
            confidences.append(confidence)

        # apply non-maxima suppression to suppress weak, overlapping bounding boxes
        idxs = cv2.dnn.NMSBoxes(boxes, confidences, self.confidence_min, self.threshold)

        # ensure at least one detection exists
        if len(idxs) > 0:
            # loop over the indexes we are keeping
            for i in idxs.flatten():
                if (boxes[i][2] * boxes[i][3] * H * W) > 50:
                    x1 = int(boxes[i][0] * W)
                    x2 = int((boxes[i][0] + boxes[i][2]) * W)
                    y1 = int(boxes[i][1] * H)
                    y2 = int((boxes[i][1] + boxes[i][3]) * H)

                    # Use segmentation data to create a mask to delete the background
                    mask = segmentation_image[y1:y2, x1:x2] == SegmentationTag.TrafficSign.color

                    # get cropped depth image
                    crop_depth = depth_image[y1:y2, x1:x2]
                    # use mask to extract the traffic sign distances
                    distance = np.average(crop_depth[mask[:, :, 1]])
                    classification = classes[i]
                    confidence = confidences[i]
                    if classification is not None and confidence > self.confidence_min:
                        detected.append(
                            DetectedObject(x=boxes[i][0], y=boxes[i][1], width=boxes[i][2], height=boxes[i][3],
                                           distance=distance,
                                           label=classification,
                                           confidence=confidence))

                    if self.data_collect_path is not None and distance < 25:
                        # get cropped rgb image
                        crop_rgb = rgb_image[y1:y2, x1:x2, :]
                        # use inverted mask to clear the background
                        crop_rgb[np.logical_not(mask)] = 255
                        now = datetime.now().strftime("%H:%M:%S-%s")
                        folder = os.path.abspath(f"{self.data_collect_path}/"
                                                 f"{classification.name if classification is not None else 'unknown'}")
                        if not os.path.exists(folder):
                            os.mkdir(folder)
                        cv2.imwrite(os.path.join(folder, f"{now}-{i}.jpg"), cv2.cvtColor(crop_rgb, cv2.COLOR_RGB2BGR))

        self.inform_listener(detected)


# Show case code
def show_image(title, image):
    max_width, max_height = 1200, 800

    limit = (max_height, max_width)
    fac = 1.0
    if image.shape[0] > limit[0]:
        fac = limit[0] / image.shape[0]
    if image.shape[1] > limit[1]:
        fac = limit[1] / image.shape[1]
    image = cv2.resize(image, (int(image.shape[1] * fac), int(image.shape[0] * fac)))
    # show the output image
    cv2.imshow(title, cv2.cvtColor(image, cv2.COLOR_RGB2BGR))
    cv2.waitKey(1)


if __name__ == "__main__":
    rospy.init_node("DetectionTest")

    detected_r = None


    def store_image(image, _):
        global detected_r
        H, W = image.shape[:2]

        if detected_r is not None:
            for element in detected_r:
                # extract the bounding box coordinates
                (x, y) = (int(element.x * W), int(element.y * H))
                (w, h) = (int(element.w * W), int(element.h * H))
                # draw a bounding box rectangle and label on the image
                color = (255, 0, 0)
                cv2.rectangle(image, (x, y), (x + w, y + h), color, 2)
                text = "{} in {:.1f}m: {:.4f}".format(element.label.label_text, element.distance, element.confidence)
                cv2.putText(image, text, (x - 5, y - 5), cv2.FONT_HERSHEY_SIMPLEX, 2, color, 4)

        # show the output image
        show_image("Traffic_sign_detection", image)


    def on_detected(detected_list):
        global detected_r
        detected_r = detected_list


    cam = RGBCamera()

    cam.set_on_image_listener(store_image)

    s = TrafficSignDetector()
    s.set_on_detection_listener(on_detected)
    rospy.spin()
