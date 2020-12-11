#!/usr/bin/env python
import os

import cv2
import rospy
import time
import numpy as np

from psaf_abstraction_layer.DepthCamera import DepthCamera

labels = None
net = None

def run():
    def on_image_update(image):
        scale_percent = 800/image.shape[1]  # percent of original size
        width = int(image.shape[1] * scale_percent)
        height = int(image.shape[0] * scale_percent)
        dim = (width, height)
        # resize image
        image= cv2.resize(image, dim, interpolation=cv2.INTER_AREA)
        over_width = (width-800.0)
        min_x = int(over_width/2)
        max_x = int(width-over_width/2)
        image=image[:,min_x:max_x]
        # to grayscale
        image /= DepthCamera.MAX_METERS
        cv2.imshow("Depth", image)
        cv2.waitKey(1)

    # Access sensor


    rospy.init_node("CameraTest")
    camera = DepthCamera()

    camera.set_on_image_listener(on_image_update)
    print("Setup done")
    rospy.spin()

if __name__ == "__main__":
    run()