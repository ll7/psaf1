import cv2
import numpy as np
import rospy
import torch

from psaf_abstraction_layer.sensors.DepthCamera import DepthCamera
from psaf_abstraction_layer.sensors.RGBCamera import RGBCamera
from psaf_perception.AbstractDetector import DetectedObject, AbstractDetector, Labels

CONV_FAC_MPH_TO_KMH = 0.621371


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
        self.labels = {'speed_30': Labels.Speed30, 'speed_60': Labels.Speed60, 'speed_90': Labels.Speed90}
        model.to(self.device)
        # Autoshape wraps model and send data already to device, but must be called after model to device
        self.net = model.autoshape()
        self.net.eval()
        torch.no_grad()  # reduce memory consumption and improve speed

        self.depth_camera = DepthCamera(role_name, "front")
        self.depth_camera.set_on_image_listener(self.__on_depth_image)
        self.rgb_camera = RGBCamera(role_name, "front")
        self.rgb_camera.set_on_image_listener(self.__on_rgb_image_update)
        self.depth_image = None

    def __on_depth_image(self, image):
        self.depth_image = image

    def __on_rgb_image_update(self, image):
        (H, W) = image.shape[:2]

        depth_image = self.depth_image  # copy depth image to ensure that the same image will be used for all calculations
        layer_outputs = self.net.forward(image)
        # initialize our lists of detected bounding boxes, confidences, and
        # class IDs, respectively
        boxes = []
        confidences = []
        class_ids = []
        for output in layer_outputs.xywhn:
            # loop over each of the detections
            for detection in output.cpu().numpy():
                # extract the class ID and confidence (i.e., probability) of
                # the current object detection
                class_id = int(detection[5])
                score = float(detection[4])
                # filter out weak predictions by ensuring the detected
                # probability is greater than the minimum probability
                if score > self.confidence_min:

                    (center_x, center_y, width, height) = detection[0:4]
                    # use the center (x, y)-coordinates to derive the top and
                    # and left corner of the bounding box
                    x = center_x - (width / 2)
                    y = center_y - (height / 2)
                    # update our list of bounding box coordinates, confidences,
                    # and class IDs
                    boxes.append([float(x), float(y), float(width), float(height)])
                    confidences.append(score)
                    class_ids.append(class_id)

        # List oif detected elements
        detected = []
        # apply non-maxima suppression to suppress weak, overlapping bounding
        # boxes
        idxs = cv2.dnn.NMSBoxes(boxes, confidences, self.confidence_min, self.threshold)

        # ensure at least one detection exists
        if len(idxs) > 0:
            # loop over the indexes we are keeping
            for i in idxs.flatten():
                if (boxes[i][2] * boxes[i][3] * H * W) > 450:
                    x1 = int(boxes[i][0] * W)
                    x2 = int((boxes[i][0] + boxes[i][2]) * W)
                    y1 = int(boxes[i][1] * H)
                    y2 = int((boxes[i][1] + boxes[i][3]) * H)
                    if depth_image is not None:
                        distance = 0.
                        crop = depth_image[y1:y2, x1:x2]
                        for _ in range(5):
                            crop = np.minimum(crop, np.average(crop))
                            # Dirty way to reduce the influence of the depth values that are not part of the
                            # sign but within in the bounding box
                            distance = np.average(crop)
                    else:
                        distance = 0.
                    detected.append(
                        DetectedObject(x=boxes[i][0], y=boxes[i][1], width=boxes[i][2], height=boxes[i][3], distance=distance,
                                       label=self.labels[self.net.names[class_ids[i]]],
                                       confidence=confidences[i]))

        self.inform_listener(detected)


# Show case code
if __name__ == "__main__":
    rospy.init_node("DetectionTest")

    detected_r = None


    def store_image(image):
        global detected_r
        H,W = image.shape[:2]

        if detected_r is not None:
            for element in detected_r:
                # extract the bounding box coordinates
                (x, y) = (int(element.x * W), int(element.y * H))
                (w, h) = (int(element.w * W), int(element.h * H))
                # draw a bounding box rectangle and label on the image
                color = (255, 0, 0)
                cv2.rectangle(image, (x, y), (x + w, y + h), color, 2)
                text = "{} in {:.1f}m: {:.4f}".format(element.label.label_text, element.distance, element.confidence)
                cv2.putText(image, text, (x - 5, y - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.75, color, 2)

        # show the output image
        cv2.imshow("RGB", cv2.cvtColor(image, cv2.COLOR_RGB2BGR))
        cv2.waitKey(1)


    def on_detected(detected_list):
        global detected_r
        detected_r = detected_list


    cam = RGBCamera()

    cam.set_on_image_listener(store_image)

    s = SpeedSignDetector()
    s.set_on_detection_listener(on_detected)
    rospy.spin()
