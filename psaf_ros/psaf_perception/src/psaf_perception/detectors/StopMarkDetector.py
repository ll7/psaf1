import csv
import os

import cv2
import rospkg
import rospy
from PyTorch_YOLOv3.models import Darknet
from PyTorch_YOLOv3.utils.utils import non_max_suppression, rescale_boxes
from PyTorch_YOLOv3.utils.datasets import Dataset, pad_to_square, resize
import torch
import torchvision.transforms as transforms

from torch.utils.data import DataLoader
from torch.autograd import Variable

from psaf_abstraction_layer.sensors.RGBCamera import RGBCamera
from psaf_perception.detectors.AbstractDetector import AbstractDetector, DetectedObject, Labels


class SingleImage(Dataset):
    """
    Wrapper to use the dataloader
    """

    def __init__(self, image, img_size=416):
        self.img_size = img_size
        self.image = image

    def __getitem__(self, index):
        img = transforms.ToTensor()(self.image)
        # Pad to square resolution
        img, _ = pad_to_square(img, 0)
        # Resize
        img = resize(img, self.img_size)

        return img

    def __len__(self):
        return 1


class StopMarkDetector(AbstractDetector):
    def __init__(self, role_name: str = "ego_vehicle", use_gpu: bool = True):
        """
        Init the speed sign detector
        :param role_name: the name of the vehicle to access the cameras
        :param use_gpu: whether the classification model should be loaded to the gpu
        """
        super().__init__()

        rospack = rospkg.RosPack()
        root_path = rospack.get_path('psaf_perception')
        self.logger_name = "StopMarkDetector"

        self.confidence_min = 0.6
        self.threshold = 0.6
        self.labels = []
        self.count = 0

        weights_path = os.path.join(root_path, "models/psaf2019_yolo.weights")
        config_path = os.path.join(root_path, "models/psaf2019_yolo.cfg")
        label_file = os.path.join(root_path, "models/psaf2019_classes.csv")
        with open(label_file, newline='') as csvfile:
            csv_in = csv.reader(csvfile, delimiter=' ', quotechar='|')
            for row in csv_in:
                self.labels.append(row[0])

        rospy.loginfo(f"init device (use gpu={use_gpu})", logger_name=self.logger_name)
        # select the gpu if allowed and a gpu is available
        self.device = torch.device("cuda:0" if use_gpu and torch.cuda.is_available() else "cpu")
        self.img_size = 416
        rospy.loginfo("loading classifier model from disk...", logger_name=self.logger_name)
        model = Darknet(config_path, self.img_size).to(self.device)
        model.load_darknet_weights(weights_path)
        model.eval()

        self.Tensor = torch.cuda.FloatTensor if torch.cuda.is_available() and use_gpu else torch.FloatTensor

        torch.no_grad()
        self.net = model

        self.rgb_camera = RGBCamera(role_name, id="front")
        self.rgb_camera.set_on_image_listener(self.__on_image_update)

    def __on_image_update(self, image, time):

        (H, W) = image.shape[:2]

        dataloader = DataLoader(
            SingleImage(image, self.img_size),
            batch_size=1,
            shuffle=False,
            num_workers=0,
        )
        output = []
        for batch_i, (input_imgs) in enumerate(dataloader):
            input_img = Variable(input_imgs.type(self.Tensor))
            input_img.to(self.device)
            output = self.net(input_img)
            output = non_max_suppression(output, self.threshold)

        # List of detected elements
        detected = []
        for detections in output:
            # loop over each of the detections
            if detections is not None:
                detections = rescale_boxes(detections, self.img_size, image.shape[:2])
                for x1, y1, x2, y2, conf, cls_conf, cls_pred in detections:
                    # extract the information
                    x1, y1, x2, y2, conf, cls_conf, cls_pred = x1.item(), y1.item(), x2.item(), y2.item(), conf.item(), cls_conf.item(), cls_pred.item()
                    class_id = int(cls_pred)
                    score = float(conf)
                    # filter out weak predictions by ensuring the detected
                    # probability is greater than the minimum probability
                    if score > self.confidence_min and self.labels[class_id] == "stop":
                        detected.append(
                            DetectedObject(float(x1 / W), float(y1 / H), (x2 - x1) / W, (y2 - y1) / H, 0,
                                           Labels.StopSurfaceMarking, score))

        self.inform_listener(time, detected)


# Show case code
if __name__ == "__main__":
    from psaf_perception.perception_util import show_image

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
                text = "{}: {:.4f}".format(element.label.label_text, element.confidence)
                cv2.putText(image, text, (x - 5, y - 5), cv2.FONT_HERSHEY_SIMPLEX, 1, color, 2)

        # show the output image
        show_image("Stopmarks",image)

    def on_detected(_,detected_list):
        global detected_r
        detected_r = detected_list


    cam = RGBCamera(id="front")

    cam.set_on_image_listener(store_image)

    s = StopMarkDetector()
    s.set_on_detection_listener(on_detected)
    rospy.spin()
