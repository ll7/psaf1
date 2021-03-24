import json
import math
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
import torch.backends.cudnn as cudnn

from psaf_abstraction_layer.sensors.FusionCamera import FusionCamera, SegmentationTag
from psaf_perception.detectors.AbstractDetector import DetectedObject, AbstractDetector, Labels


class TrafficLightDetector(AbstractDetector):
    """
    Detector for traffic lights
    """

    def __init__(self, role_name: str = "ego_vehicle", use_gpu: bool = True):
        """
        Init the speed sign detector
        :param role_name: the name of the vehicle to access the cameras
        :param use_gpu: whether the classification model should be loaded to the gpu
        """
        super().__init__()

        self.logger_name = "TrafficLightDetector"

        self.confidence_min = 0.65
        self.threshold = 0.7
        self.canny_threshold = 100

        self.data_collect_path = None  # "/home/psaf1/Documents/traffic_light_data"

        rospack = rospkg.RosPack()
        root_path = rospack.get_path('psaf_perception')

        # Deep learning config and inits
        self.confidence_min = 0.70
        self.threshold = 0.7
        rospy.loginfo(f"init device (use gpu={use_gpu})", logger_name=self.logger_name)
        # select the gpu if allowed and a gpu is available
        self.device = torch.device("cuda:0" if use_gpu and torch.cuda.is_available() else "cpu")
        rospy.loginfo("Device:" + str(self.device), logger_name=self.logger_name)
        # load our model
        model_name = 'traffic-light-classifiers-2021-02-09-18:29:47'
        rospy.loginfo("loading classifier model from disk...", logger_name=self.logger_name)
        model = torch.load(os.path.join(root_path, f"models/{model_name}.pt"))

        with open(os.path.join(root_path, f"models/{model_name}.names")) as f:
            class_names = json.load(f)
        self.labels = {
            class_names['back']: None,
            class_names['green']: Labels.TrafficLightGreen,
            class_names['red']: Labels.TrafficLightRed,
            class_names['yellow']: Labels.TrafficLightYellow
        }
        self.transforms = transforms.Compose([
            transforms.ToPILImage(),
            transforms.Resize((224, 224)),
            transforms.ToTensor(),
            transforms.Normalize([0.485, 0.456, 0.406], [0.229, 0.224, 0.225])
        ])
        model.to(self.device)
        if self.device.type == 'cuda':
            rospy.loginfo("Enable cudnn", logger_name=self.logger_name)
            cudnn.enabled = True
            cudnn.benchmark = True
        self.net = model
        self.net.eval()
        torch.no_grad()  # reduce memory consumption and improve speed

        # init image source = combination of segmentation, rgb and depth camera
        self.combinedCamera = FusionCamera(role_name=role_name,visible_tags={SegmentationTag.TrafficLight})
        self.combinedCamera.set_on_image_listener(self.__on_new_image_data)

    def __extract_label(self, image) -> Tuple[Labels, float]:
        """
        Analyze the given image of the traffic light and returns the corresponding label
        :param image: the important part of the camera image
        :return: the Label
        """
        image = self.transforms(image).unsqueeze(dim=0)
        imgblob = Variable(image).to(self.device)
        pred = torch.nn.functional.softmax(self.net(imgblob).cpu(), dim=1).detach().numpy()[0, :]

        hit = np.argmax(pred)
        label = self.labels.get(hit, Labels.TrafficLightUnknown)

        return label, float(pred[hit])

    def __on_new_image_data(self,time, segmentation_image, rgb_image, depth_image):
        """
        Handles the new image data from the cameras
        :param segmentation_image: the segmentation image
        :param rgb_image: the rgb image
        :param depth_image: the depth data
        :param time: time stamp when the images where taken
        :return: none
        """
        if (rospy.Time.now()-time).to_sec()>0.6:
            # Ignore the new image data if its older than 600ms
            # -> Reduce message aging
            return 
        (height_seg, width_seg) = segmentation_image.shape[:2]
        (height_rgb, width_rgb) = rgb_image.shape[:2]

        # List oif detected elements
        detected = []

        canny_output = cv2.Canny(segmentation_image, self.canny_threshold, self.canny_threshold * 2)
        contours, _ = cv2.findContours(canny_output, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        boxes = []
        classes = []
        confidences = []
        distances = []

        for i, c in enumerate(contours):
            contours_poly = cv2.approxPolyDP(c, 3, True)
            x1, y1, w, h = cv2.boundingRect(contours_poly)
            x2 = x1 + w
            y2 = y1 + h
            if w < h and w > 1 and h > 2:
                mask = segmentation_image[y1:y2, x1:x2] != (255,255,255)

                # get cropped depth image
                crop_depth = depth_image[y1:y2, x1:x2]
                masked_crop_depth = crop_depth[mask[:, :, 1]]

                # use mask to extract the traffic sign distances
                distance = np.min(masked_crop_depth)

                if distance <= 100:
                    h_scale = width_rgb/width_seg
                    v_scale = height_rgb/height_seg
                    # get cropped rgb image
                    crop_rgb = rgb_image[
                               int(y1 * v_scale):min([height_rgb, int(y2 * v_scale + 1)]),
                               int(x1 * h_scale):min([width_rgb, int(x2 * h_scale + 1)]),
                               :]
                    # Classify the cropped image
                    label, confidence = self.__extract_label(crop_rgb)
                    if label is not None:
                        boxes.append(np.array([x1, y1, w, h]) / np.array([width_seg, height_seg, width_seg, height_seg]))
                        classes.append(label)
                        confidences.append(confidence)
                        distances.append(distance)

        # apply non-maxima suppression to suppress weak, overlapping bounding boxes
        idxs = cv2.dnn.NMSBoxes(boxes, confidences, self.confidence_min, self.threshold)

        # ensure at least one detection exists
        if len(idxs) > 0:
            # loop over the indexes we are keeping
            for i in idxs.flatten():
                distance = distances[i]
                label = classes[i]
                confidence = confidences[i]

                if not confidence >= self.confidence_min:
                    label = Labels.TrafficLightUnknown
                    confidence = 1.

                detected.append(
                    DetectedObject(x=boxes[i][0], y=boxes[i][1], width=boxes[i][2], height=boxes[i][3],
                                   distance=distance,
                                   label=label,
                                   confidence=confidence))
                # Store traffic light data in folder to train a better network
                if self.data_collect_path is not None and distance < 25:
                    x1, y1, w, h = boxes[i]
                    x1 = int(x1 * h_scale)
                    y1 = int(y1 * v_scale)
                    x2 = min([width_rgb, int((x1 + w) * h_scale + 1)])
                    y2 = min([height_rgb, int((y1 + h) * v_scale + 1)])
                    # get cropped rgb image
                    crop_rgb = rgb_image[y1:y2, x1:x2, :]
                    # use inverted mask to clear the background
                    crop_rgb[np.logical_not(mask)] = 0
                    now = datetime.now().strftime("%H:%M:%S")
                    folder = os.path.abspath(
                        f"{self.data_collect_path}/{label.name if label is not None else 'unknown'}")
                    if not os.path.exists(folder):
                        os.mkdir(folder)
                    cv2.imwrite(os.path.join(folder, f"{now}-{i}.jpg"), cv2.cvtColor(crop_rgb, cv2.COLOR_RGB2BGR))

        self.inform_listener(time,detected)


# Show case code
if __name__ == "__main__":
    from psaf_abstraction_layer.sensors.RGBCamera import RGBCamera
    from psaf_perception.perception_util import show_image


    rospy.init_node("DetectionTest")

    detected_r = None
    time = None


    def store_image(image, _):
        global detected_r,time

        H, W = image.shape[:2]

        if detected_r is not None:
            for element in detected_r:
                # extract the bounding box coordinates
                (x, y) = (int(element.x * W), int(element.y * H))
                (w, h) = (int(element.w * W), int(element.h * H))
                # draw a bounding box rectangle and label on the image
                color_map = {Labels.TrafficLightRed: (255, 0, 0), Labels.TrafficLightGreen: (0, 255, 0),
                             Labels.TrafficLightYellow: (255, 255, 0),
                             Labels.TrafficLightUnknown: (0, 0, 0)}
                color = color_map.get(element.label, (0, 0, 0))
                cv2.rectangle(image, (x, y), (x + w, y + h), color, 2)
                text = "{}-{:.1f}m: {:.4f}".format(element.label.label_text, element.distance, element.confidence)
                cv2.putText(image, text, (x - 5, y - 5), cv2.FONT_HERSHEY_SIMPLEX, 1, color, 2)

            print(f"Age {(rospy.Time.now()-time).to_sec()}")
        show_image("Traffic light detection", image)


    def on_detected(time_in,detected_list):
        global detected_r,time
        detected_r = detected_list
        time = time_in


    cam = RGBCamera()

    cam.set_on_image_listener(store_image)

    s = TrafficLightDetector()
    s.set_on_detection_listener(on_detected)
    rospy.spin()
