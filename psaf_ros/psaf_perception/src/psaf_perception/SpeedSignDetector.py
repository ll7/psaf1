from datetime import datetime
import os

import cv2
import rospkg
import rospy
import numpy as np
from psaf_abstraction_layer.sensors.RGBCamera import RGBCamera
from psaf_perception.AbstractDetector import Labels, AbstractDetector, DetectedObject
from psaf_perception.CameraDataFusion import CameraDataFusion, SegmentationTag

CONV_FAC_MPH_TO_KMH = 0.621371


class SpeedSignDetector(AbstractDetector):

    def __init__(self, role_name: str = "ego_vehicle"):
        super().__init__()

        rospack = rospkg.RosPack()
        root_path = rospack.get_path('psaf_perception')
        self.descriptor_path = os.path.join(root_path, "models/speed_descriptors")

        self.confidence_min = 0.70
        self.threshold = 0.7
        self.canny_threshold = 100

        self.labels = {'speed_limit_30': Labels.SpeedLimit30, 'speed_30': Labels.Speed30, 'speed_60': Labels.Speed60,
                       'speed_90': Labels.Speed90}

        self.descriptors = {}
        self.__load_descriptors()

        self.collect_unlabeled_data = False

        # init image source = combination of segmentation, rgb and depth camera
        self.combinedCamera = CameraDataFusion(role_name=role_name, time_threshold=0.08,
                                               visible_tags=set([SegmentationTag.TrafficSign]))
        self.combinedCamera.set_on_image_data_listener(self.__on_new_image_data)

    def __load_descriptors(self):
        self.descriptors.clear()

        label_folders = []
        for folder in os.listdir(self.descriptor_path):
            folder = os.path.join(self.descriptor_path,folder)
            if os.path.isdir(folder):
                label = os.path.basename(folder)
                if label in self.labels.keys():
                    label_folders.append((folder,label))

        for folder,label in label_folders:
            self.descriptors[label] = []
            for image_path in os.listdir(folder):
                img = cv2.imread(os.path.join(folder,image_path))
                kp_goal, des_goal = self.__compute_descriptor(img)
                self.descriptors[label].append(des_goal)

    def __compute_descriptor(self,image):
        gray = cv2.cvtColor(image, cv2.COLOR_RGB2GRAY)
        gray = cv2.GaussianBlur(gray, (5, 5), 0)

        orb = cv2.ORB_create(patchSize=12,edgeThreshold=20)

        return orb.detectAndCompute(gray, None)

    def __classify(self, image):


        kp_curr, des_curr = self.__compute_descriptor(image)

        matcher = cv2.BFMatcher()
        if not (des_curr is None):
            max_label_count = {}
            for label, descriptor_list in self.descriptors.items():
                max_label_count[label] = 0
                for des_goal in descriptor_list:
                    matches = matcher.knnMatch(des_goal, des_curr, k=2)
                    good = []
                    for i, values in enumerate(matches):
                        if len(values) == 2: # ensure that match contains enough values
                            (curr, goal) = values
                            if curr.distance < 0.75 * goal.distance: # filter for "good" matches
                                good.append([curr])

                    if len(good) > max_label_count[label]: # store only the best match for the current label
                        max_label_count[label] = len(good)

                print(f"{label}: score={max_label_count[label]}")
            # get best label
            best_label = max(max_label_count.items(), default=("None",0), key=lambda x: x[1])
            if best_label[1] >= 5:
                return self.labels.get( best_label[0], None)
        return None

    def __on_new_image_data(self, segmentation_image, rgb_image, depth_image, time):
        (H, W) = segmentation_image.shape[:2]

        # List oif detected elements
        detected = []

        canny_output = cv2.Canny(segmentation_image, self.canny_threshold, self.canny_threshold * 2)
        contours, _ = cv2.findContours(canny_output, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        boxes = []
        for i, c in enumerate(contours):
            contours_poly = cv2.approxPolyDP(c, 3, True)
            boxes.append(cv2.boundingRect(contours_poly) / np.array([W, H, W, H]))

        confidences = [1.] * len(boxes)
        # apply non-maxima suppression to suppress weak, overlapping bounding
        # # boxes
        idxs = cv2.dnn.NMSBoxes(boxes, confidences, self.confidence_min, self.threshold)

        # ensure at least one detection exists
        if len(idxs) > 0:
            # loop over the indexes we are keeping
            for i in idxs.flatten():
                if (boxes[i][2] * boxes[i][3] * H * W) > 100:
                    x1 = int(boxes[i][0] * W)
                    x2 = int((boxes[i][0] + boxes[i][2]) * W)
                    y1 = int(boxes[i][1] * H)
                    y2 = int((boxes[i][1] + boxes[i][3]) * H)

                    distance = 0.
                    crop_depth = depth_image[y1:y2, x1:x2]
                    for _ in range(5):
                        crop_depth = np.minimum(crop_depth, np.average(crop_depth))
                        # Dirty way to reduce the influence of the depth values that are not part of the
                        # sign but within in the bounding box
                        distance = np.average(crop_depth)

                    crop_rgb = rgb_image[y1:y2, x1:x2, :]
                    classification = self.__classify(crop_rgb)
                    if classification is not None:
                        detected.append(
                            DetectedObject(x=boxes[i][0], y=boxes[i][1], width=boxes[i][2], height=boxes[i][3],
                                           distance=distance,
                                           label=classification,
                                           confidence=confidences[i]))
                    elif self.collect_unlabeled_data:
                        now = datetime.now().strftime("%H:%M:%S")
                        cv2.imwrite(os.path.join(self.descriptor_path,f"unlabeled/{now}.jpg"),cv2.cvtColor(crop_rgb, cv2.COLOR_RGB2BGR))

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
        show_image("Speed_detection", image)


    def on_detected(detected_list):
        global detected_r
        detected_r = detected_list


    cam = RGBCamera()

    cam.set_on_image_listener(store_image)

    s = SpeedSignDetector()
    s.set_on_detection_listener(on_detected)
    rospy.spin()
