
import os
import csv

import cv2
import numpy as np
from PIL import Image
from pathlib import Path
import torch
import torch.nn as nn
import torch.nn.functional as F
from torch.utils.data import Dataset, DataLoader
from torchvision import transforms
from sklearn.model_selection import KFold
import sklearn.metrics as metrics
from skimage import io
import time
from tqdm import tqdm

def run():
    source_dir = "/home/psaf1/project-files/training_data/carlaSpeedSigns/dataset_CARLA"

    filelist = []

    for file in os.listdir(source_dir):
        if file.endswith(".txt"):
            filelist.append(os.path.join(source_dir,file))
    print(f"Experiment using files from {source_dir}")
    print("Loading Data...")

    all_samples = []
    for fpath in filelist:
        with open(fpath, newline='') as csvfile:
            csv_in = csv.reader(csvfile, delimiter=' ', quotechar='|')
            for row in csv_in:
                label = None
                if row[0] == '0':
                    label = 0
                elif row[0] == '1':
                    label = 1
                elif row[0] == '2':
                    label = 2
                all_samples.append([file, label, [row[1:5]]])

    print("Loading Data... finished.")

    all_samples = np.array(all_samples)

    """New Dataset"""

    class SpeedSignDataset(Dataset):

        def __init__(self, samples, basic_transform=None, data_augmentation=None):
            self.samples = samples
            self.data_augmentation = data_augmentation
            self.basic_transform = basic_transform

        def __len__(self):
            return len(self.samples)

        def __getitem__(self, index):
            image = cv2.imread(self.samples[index][0])[:, :, ::-1]
            class_label = self.samples[index][1]
            bounding_box = [self.samples[index][2]]
            x_1 = bounding_box[0] * image.width
            y_1 = bounding_box[1] * image.height
            x_2 = bounding_box[2] * image.width
            y_2 = bounding_box[3] * image.height

            if self.data_augmentation:
                image = self.data_augmentation(image)

            if self.basic_transform:
                image = self.basic_transform(image)

            labels = [x_1,y_1,x_2,y_2,1.0,class_label]
            sample = {'image': image, 'labels': torch.Tensor(labels)}
            return sample

    def train(model, data_generator, epochs, optimizer, loss_criterion, device):

        print("Starting training")
        for e in tqdm(range(epochs)):
            for batch in data_generator:
                data = batch['sensorvalues']
                labels = batch['labels']
                data = data.to(device)
                labels = labels.to(device)
                optimizer.zero_grad()  # reset optimizer
                output = model(data)  # forward pass
                # Change output to fit to training data values have to be in between 0 and 1
                output = nn.Sigmoid()(output)

                loss = loss_criterion(output, labels)  # loss calculation
                loss.backward()  # backpropagation of loss
                optimizer.step()  # next optimizer step

    # Test method
    def test_model(model, data_generator, device):
        print("Starting testing")
        all_preds = np.array([])
        all_labels = np.array([])
        model.eval()  # set model mode to eval
        for test_batch in tqdm(test_dataloader):
            with torch.no_grad():
                data = test_batch['sensorvalues']
                labels = test_batch['labels']
                data = data.to(device)
                labels = labels.to(device)
                output = model(data)
                # Change output to fit to training data values have to be in between 0 and 1
                output = nn.Sigmoid()(output)
                pred, y = output.detach().cpu().numpy(), labels.detach().cpu().numpy()
                all_preds = np.append(all_preds, pred)
                all_labels = np.append(all_labels, y)
        model.train()  # set model mode to train again

        return all_preds, all_labels

    ### Hyperparameters ###
    train_batchsize = 256
    test_batchsize = 256
    epochs = 10
    #######################

    # Transform data to be used with pretrained model
    basic_transform = transforms.Compose([
        transforms.ToPILImage(),
        transforms.Resize(224),
        transforms.ToTensor()  # ,
        # transforms.Normalize(mean=[0.5, ], std=[0.5,])
    ])

    def getPretrainedModel():
        pretrained_model = torch.hub.load('ultralytics/yolov5', 'yolov5s', pretrained=True, classes=3).fuse().autoshape()

        return pretrained_model

    # Prepare split
    kf = KFold(n_splits=5, shuffle=False)
    kf.get_n_splits(all_samples)
    cv_scores = []

    if torch.cuda.is_available():
        torch.cuda.empty_cache()

    device = torch.device("cuda:0" if torch.cuda.is_available() else "cpu")  # for using the GPU in pytorch
    print("Device:" + str(device))
    loss_criterion = nn.BCELoss()

    start_time = time.time()
    for fold_index, (train_index, test_index) in enumerate(kf.split(all_samples)):
        print(f"##### Fold {fold_index + 1} #####")

        model = getPretrainedModel()
        model.to(device)
        # Observe that all parameters are being optimized
        optimizer_ft = torch.optim.Adam(model.parameters(), lr=0.00001)

        train_samples = np.vstack(all_samples[train_index])
        test_samples = np.vstack(all_samples[test_index])

        trainset = SpeedSignDataset(train_samples, basic_transform=basic_transform, data_augmentation=my_data_augmentation)
        train_dataloader = DataLoader(trainset, batch_size=train_batchsize, shuffle=True)
        train(model, train_dataloader, epochs, optimizer_ft, loss_criterion, device)

        testset = SpeedSignDataset(test_samples, basic_transform=basic_transform)
        test_dataloader = DataLoader(testset, batch_size=test_batchsize, shuffle=True)

        all_preds, all_labels = test_model(model, test_dataloader, device)

        all_preds = np.around(all_preds)
        accuracy_score = metrics.accuracy_score(all_labels, all_preds)
        print(f"Accuracy on fold {fold_index + 1}: {accuracy_score}")
        cv_scores.append(accuracy_score)

    taken = round(time.time() - start_time, 2)
    print(f"Time taken: {taken} sec\n")

    """Give me the damn accuracy"""

    print(f"Accuracy: {round(np.mean(cv_scores) * 100, 2)}% (+/- {round(np.std(cv_scores) * 2 * 100, 2)}%)")
    print(f"Fold scores: {[round(score * 100, 2) for score in cv_scores]}")


if __name__ == "__main__":
    run()