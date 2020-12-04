#!/usr/bin/env python
import os

import cv2
import rospy
import time
import numpy as np

from psaf_abstraction_layer.RGBCamera import RGBCamera

labels = None
net = None

def run():
    def on_image_update(image):
        (H, W) = image.shape[:2]

        confidence_min = 0.6
        threshold = 0.9

        blob = cv2.dnn.blobFromImage(image, 1 / 255.0, (416, 416),
                                     swapRB=True, crop=False)
        net.setInput(blob)
        start = time.time()
        layerOutputs = net.forward(ln)
        end = time.time()
        # show timing information on YOLO
        print("[INFO] YOLO took {:.6f} seconds".format(end - start))
        # initialize our lists of detected bounding boxes, confidences, and
        # class IDs, respectively
        boxes = []
        confidences = []
        classIDs = []  # loop over each of the layer outputs
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
                if confidence > confidence_min:
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

        # apply non-maxima suppression to suppress weak, overlapping bounding
        # boxes
        idxs = cv2.dnn.NMSBoxes(boxes, confidences, confidence, threshold)
        # ensure at least one detection exists
        if len(idxs) > 0:
            # loop over the indexes we are keeping
            for i in idxs.flatten():
                # extract the bounding box coordinates
                (x, y) = (boxes[i][0], boxes[i][1])
                (w, h) = (boxes[i][2], boxes[i][3])
                # draw a bounding box rectangle and label on the image
                color = [int(c) for c in COLORS[classIDs[i]]]
                cv2.rectangle(image, (x, y), (x + w, y + h), color, 2)
                text = "{}: {:.4f}".format(classIDs[i], confidences[i])
                cv2.putText(image, text, (x, y - 5), cv2.FONT_HERSHEY_SIMPLEX,
                            0.5, color, 2)
        # show the output image
        cv2.imshow("RGB", image)
        cv2.waitKey(1)

    np.random.seed(42)
    COLORS = np.random.randint(0, 255, size=(88, 3),
                               dtype="uint8")

    # derive the paths to the YOLO weights and model configuration
    weightsPath = os.path.abspath("../../../models/yolov3.weights")
    configPath = os.path.abspath("../../../models/yolov3.cfg")
    # load our YOLO object detector trained on COCO dataset (80 classes)W
    print("[INFO] loading YOLO from disk...")
    net = cv2.dnn.readNetFromDarknet(configPath, weightsPath)

    # determine only the *output* layer names that we need from YOLO
    ln = net.getLayerNames()
    ln = [ln[i[0] - 1] for i in net.getUnconnectedOutLayers()]
    print(ln)


    # Access sensor


    rospy.init_node("CameraTest")
    camera = RGBCamera()

    camera.set_on_image_listener(on_image_update)
    rospy.spin()

if __name__ == "__main__":
    run()