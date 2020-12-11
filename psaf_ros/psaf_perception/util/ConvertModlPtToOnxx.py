#!/usr/bin/env python

import argparse

import cv2
import torch


if __name__ == "__main__":

    parser = argparse.ArgumentParser()
    parser.add_argument('--model_type', type=str, default='yolov5s', help='the model path')
    parser.add_argument('--weights', type=str, default='model.pt', help='initial weights path')
    parser.add_argument('--out', type=str, default='model.onxx', help='output path')
    opt = parser.parse_args()
    # model = torch.hub.load('ultralytics/yolov5', opt.model_type, pretrained=False, classes=3)
    # model.load_state_dict(torch.load(opt.weights))
    model = torch.load(opt.weights);
    # Input to the model
    batch_size = 1  # just a random number

    # get random input
    print(f'Downloading example input..')
    torch.hub.download_url_to_file('https://github.com/ultralytics/yolov5/releases/download/v1.0/' + 'zidane.jpg', 'zidane.jpg')
    img1 = cv2.imread('zidane.jpg')[:, :, ::-1]
    x = [img1]
    torch_out = model(x, size=640)

    # Export the model
    torch.onnx.export(model,  # model being run
                      x,  # model input (or a tuple for multiple inputs)
                      opt.out,  # where to save the model (can be a file or file-like object)
                      export_params=True,  # store the trained parameter weights inside the model file
                      opset_version=10,  # the ONNX version to export the model to
                      do_constant_folding=True,  # whether to execute constant folding for optimization
                      input_names=['input'],  # the model's input names
                      output_names=['output'],  # the model's output names
                      dynamic_axes={'input': {0: 'batch_size'},  # variable lenght axes
                                    'output': {0: 'batch_size'}})