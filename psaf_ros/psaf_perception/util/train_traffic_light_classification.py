from __future__ import division
from __future__ import print_function

import copy
import json
import os
import time
from datetime import datetime

import torch
import torch.nn as nn
import torch.optim as optim
from torchvision import datasets, models, transforms
from tqdm import tqdm

# Mainly inspired by the pytorch fine tuning tutorial

# Top level data directory. Here we assume the format of the directory conforms
#   to the ImageFolder structure
data_dir = "/home/psaf1/project-files/training_data/traffic_light_data"

# Generate path where the model will be stored
now = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
store_path =os.path.abspath(f"../models/traffic-light-classifiers-{now}")
# Use existing model that will be fine tuned
# set 'alexnet', 'resnet' or path to an existing pt file
base_model_name = "alexnet"
# None if net should be trained based on standard pretrained network -> define in base_model_name
existing_model_path = "../models/traffic-light-classifiers-2021-03-26_12-49-07"

# classes in the dataset
classes = ['back', 'green','red','yellow']

# Batch size for training (change depending on how much memory you have)
batch_size = 256

# Number of epochs to train for
num_epochs = 500

# Flag for feature extracting. When False, we finetune the whole model,
#   when True we only update the reshaped layer params
feature_extract = False

# Use cudnn for speed up
use_cudnn = True


def train_model(model, dataloaders, criterion, optimizer, num_epochs=25):
    since = time.time()

    val_acc_history = []

    best_model_wts = copy.deepcopy(model.state_dict())
    best_acc = 0.0

    for epoch in tqdm(range(num_epochs), desc="Train..."):
        # Each epoch has a training and validation phase
        for phase in ['train', 'val']:
            if phase == 'train':
                model.train()  # Set model to training mode
            else:
                model.eval()  # Set model to evaluate mode

            running_loss = 0.0
            running_corrects = 0

            # Iterate over data.
            for (inputs, labels) in dataloaders[phase]:
                inputs = inputs.to(device)
                labels = labels.to(device)

                # zero the parameter gradients
                optimizer.zero_grad()

                # forward
                # track history if only in train
                with torch.set_grad_enabled(phase == 'train'):
                    # Get model outputs and calculate loss
                    # Special case for inception because in training it has an auxiliary output. In train
                    #   mode we calculate the loss by summing the final output and the auxiliary output
                    #   but in testing we only consider the final output.
                    outputs = model(inputs)
                    loss = criterion(outputs, labels)

                    _, preds = torch.max(outputs, 1)

                    # backward + optimize only if in training phase
                    if phase == 'train':
                        loss.backward()
                        optimizer.step()

                # statistics
                running_loss += loss.item() * inputs.size(0)
                running_corrects += torch.sum(preds == labels.data)

            epoch_loss = running_loss / len(dataloaders[phase].dataset)
            epoch_acc = running_corrects.double() / len(dataloaders[phase].dataset)

            # tqdm.write('{} Loss: {:.4f} Acc: {:.4f}'.format(phase, epoch_loss, epoch_acc))

            # deep copy the model
            if phase == 'val' and epoch_acc > best_acc:
                best_acc = epoch_acc
                best_model_wts = copy.deepcopy(model.state_dict())
            if phase == 'val':
                val_acc_history.append(epoch_acc)

    time_elapsed = time.time() - since
    print('-' * 20)
    print('Training complete in {:.0f}m {:.0f}s'.format(time_elapsed // 60, time_elapsed % 60))
    print('Best val Acc: {:4f}'.format(best_acc))
    print('-' * 20)

    # load best model weights
    model.load_state_dict(best_model_wts)
    return model, val_acc_history


def set_parameter_requires_grad(model, feature_extracting):
    if feature_extracting:
        for param in model.parameters():
            param.requires_grad = False


def initialize_model(num_classes, feature_extract, use_pretrained=True):
    global base_model_name
    # Initialize these variables which will be set in this if statement. Each of these
    #   variables is model specific.

    """ Resnet
    """
    if existing_model_path is None:
        if base_model_name == "resnet":
            model_ft = models.resnet18(pretrained=use_pretrained)
            set_parameter_requires_grad(model_ft, feature_extract)
            num_ftrs = model_ft.fc.in_features
            model_ft.fc = nn.Linear(num_ftrs, num_classes)
        elif base_model_name == "alexnet":
            # Alexnet
            model_ft = models.alexnet(pretrained=use_pretrained)
            set_parameter_requires_grad(model_ft, feature_extract)
            num_ftrs = model_ft.classifier[6].in_features
            model_ft.classifier[6] = nn.Linear(num_ftrs, num_classes)
        else:
            assert False
    else:
        with open(os.path.abspath(existing_model_path+".config")) as f:
            config = json.load(f)
            base_model_name = config.get("base model name","unknown")
        model_ft = torch.load(os.path.abspath(existing_model_path+".pt"))
        set_parameter_requires_grad(model_ft, feature_extract)

    input_size = 224

    return model_ft, input_size


if __name__ == "__main__":

    model_ft, input_size = initialize_model(len(classes), feature_extract, use_pretrained=True)

    # Just normalization for validation
    data_transforms = {
        'train': transforms.Compose([
            transforms.Resize((input_size,input_size)),
            transforms.ColorJitter(brightness=0.1, contrast=0.2, saturation=0.2, hue=0.05),
            transforms.RandomHorizontalFlip(),
            transforms.RandomPerspective(distortion_scale=0.5, p=0.5, interpolation=2, fill=0),
            transforms.ToTensor(),
            transforms.Normalize([0.485, 0.456, 0.406], [0.229, 0.224, 0.225])
        ]),
        'val': transforms.Compose([
            transforms.Resize(input_size),
            transforms.CenterCrop(input_size),
            transforms.ToTensor(),
            transforms.Normalize([0.485, 0.456, 0.406], [0.229, 0.224, 0.225])
        ]),
    }

    print("Initializing Datasets and Dataloaders...")


    def filter_classes(string: str) -> bool:
        """
        Filter the data to train only the given classes
        :param string:
        :return:
        """
        return any(class_name in string for class_name in classes)


    # Create training and validation datasets
    image_datasets = {
        x: datasets.ImageFolder(os.path.join(data_dir, x), data_transforms[x], is_valid_file=filter_classes) for x in
        ['train', 'val']}
    # Create training and validation dataloaders
    dataloaders_dict = {
        x: torch.utils.data.DataLoader(image_datasets[x], batch_size=batch_size, shuffle=True, num_workers=4) for x in
        ['train', 'val']}

    # Detect if we have a GPU available
    device = torch.device("cuda:0" if torch.cuda.is_available() else "cpu")

    if device.type == 'cuda':
        print("Enable cudnn")
        torch.backends.cudnn.enabled = True
        torch.backends.cudnn.benchmark = True
    # Send the model to GPU
    model_ft = model_ft.to(device)

    # Gather the parameters to be optimized/updated in this run. If we are
    #  finetuning we will be updating all parameters. However, if we are
    #  doing feature extract method, we will only update the parameters
    #  that we have just initialized, i.e. the parameters with requires_grad
    #  is True.
    params_to_update = model_ft.parameters()
    print("Params to learn:")
    if feature_extract:
        params_to_update = []
        for name, param in model_ft.named_parameters():
            if param.requires_grad == True:
                params_to_update.append(param)
                print("\t", name)
    else:
        for name, param in model_ft.named_parameters():
            if param.requires_grad == True:
                print("\t", name)

    # Observe that all parameters are being optimized
    optimizer_ft = optim.SGD(params_to_update, lr=0.001, momentum=0.9)

    # Setup the loss fxn
    criterion = nn.CrossEntropyLoss()

    print(f"Start training with {len(image_datasets['train'])} images.")

    # Train and evaluate
    model_ft, hist = train_model(model_ft, dataloaders_dict, criterion, optimizer_ft, num_epochs=num_epochs)

    print(f"Save model to '{store_path}'...")
    torch.save(model_ft, f"{store_path}.pt")
    names = image_datasets['train'].class_to_idx
    with open(f"{store_path}.names", 'w') as f:
        json.dump(names, f)
    results = {
        'epochs': num_epochs,
        'base model name': base_model_name,
        'training_based_on': existing_model_path,
        'batch size': batch_size,
        'freeze feature extraction layers': feature_extract,
        'Used cudnn': use_cudnn,
        'validation_accuracy': float(max(hist).cpu()),
        'number of training images': len(image_datasets['train']),
        'number of validation images': len(image_datasets['val']),

    }
    with open(f"{store_path}.config", 'w') as f:
        json.dump(results, f)

    print("...Done saving")