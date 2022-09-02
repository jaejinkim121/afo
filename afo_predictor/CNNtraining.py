# -*- coding: utf-8 -*-
"""
Created on Sun Aug 21 16:55:23 2022

@author: minhee
"""
import os
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from tqdm import tqdm

import torch
import torch.nn as nn
import torch.nn.functional as F
from torchvision import transforms
from torch.utils.data import Dataset, DataLoader, random_split


from include.CNNmodel import *
from include.utils import *


def CNNtraining(data_dir, DEVICE, model_dir=None,
                BATCH_SIZE=128, EPOCHS=35, TRAIN_SIZE=0.85, LR=0.00001,
                N_WARMUP_STEPS=5, DECAY_RATE=0.98, INPUT_LENGTH=15):
    dataset = CustomDataset(input_length=INPUT_LENGTH, data_dir=data_dir)

    dataset_size = len(dataset)
    train_size = int(dataset_size * TRAIN_SIZE)
    test_size = dataset_size - train_size
    train_dataset, test_dataset = random_split(
        dataset, [train_size, test_size])

    # dataloader = DataLoader(dataset, batch_size=BATCH_SIZE, shuffle=True,
    #                         collate_fn=dataset.collate_fn, drop_last=True)
    train_dataloader = DataLoader(train_dataset, batch_size=BATCH_SIZE,
                                  shuffle=True, drop_last=True,
                                  collate_fn=dataset.collate_fn)
    test_dataloader = DataLoader(test_dataset, batch_size=BATCH_SIZE,
                                  shuffle=True, drop_last=True,
                                  collate_fn=dataset.collate_fn)

    DEVICE = torch.device('cuda') \
        if torch.cuda.is_available else torch.device('cpu')
    print("Using PyTorch version: {}, Device: {}".format(
        torch.__version__, DEVICE))

    model = Conv1DNet().to(DEVICE)
    optimizer = torch.optim.Adam(model.parameters(), LR)
    scheduler = ScheduleOptim(optimizer, N_WARMUP_STEPS, DECAY_RATE)
    # criterion = nn.HuberLoss()
    criterion = RMSELoss()

    patience = 0
    best_loss = 1000

    for epoch in range(1, EPOCHS + 1):
        train_loss = train(model, train_dataloader, scheduler,
                           epoch, criterion, DEVICE)
        test_loss = evaluate(model, test_dataloader, criterion, DEVICE)
        lr = scheduler.get_lr()
        print("\n[EPOCH: {:2d}], \tModel: Conv1DNet, \tLR: {:8.5f}, ".format(
            epoch, lr) \
            + "\tTrain Loss: {:8.3f}, \tTest Loss: {:8.3f} \n".format(
                train_loss, test_loss))
        # Early stopping
        if test_loss < best_loss:
            best_loss = test_loss
            patience = 0
        else:
            patience += 1
            if patience >= 5:
                break

    # CNN model save
    if model_dir is not None:
        torch.save(model.state_dict(), model_dir)

    return test_loss

# hyperparameter settings
BATCH_SIZE = 32
EPOCHS = 50
TRAIN_SIZE = 0.85
LR = 0.001
N_WARMUP_STEPS = 5
DECAY_RATE = 0.98
INPUT_LENGTH = 15 #1, 3, 5, 10, 13, 15

# device setting
DEVICE = torch.device('cuda') \
    if torch.cuda.is_available else torch.device('cpu')
print("Using PyTorch version: {}, Device: {}".format(
    torch.__version__, DEVICE))

# define calib data path and model path
calib_data_path = "./data/280/Calibration/"
model_path = "./data/280/CNN_model/"
try:
    if not os.path.exists(model_path):
        os.makedirs(model_path)
except:
    pass

# for loop for 12 sensor training
_, sensor_name_list = folder_path_name(calib_data_path)

df_loss = pd.DataFrame(columns=["sensor_name", "LR", "loss"])
# for LR in [100, 10, 1, 0.1, 0.01, 0.001, 0.0001, 0.00001, 0.00001]:

for n, name in enumerate(sensor_name_list):

    LR_list = [0.0011,0.0011,0.011,0.0009,0.001,0.0012,
               0.001,0.09,0.011,0.0011,0.011,0.001]
    lr = LR_list[n]
    
    # for lr in [LR-LR/5, LR-LR/10, LR, LR+LR/10, LR+LR/5]:
        # if name.endswith("Right_4"):
    print("START 1D CNN MODEL TRAINING!!! %s" % name)
    print("LR: %f" % lr)
    data_dir = calib_data_path + name + "/force_conversion_test.csv"
    model_dir = model_path + name + ".pt"
    final_test_loss = CNNtraining(data_dir, DEVICE,
                                  model_dir=model_dir,
                                  BATCH_SIZE=BATCH_SIZE, EPOCHS=EPOCHS,
                                  LR=lr, N_WARMUP_STEPS=N_WARMUP_STEPS,
                                  DECAY_RATE=DECAY_RATE,
                                  INPUT_LENGTH=INPUT_LENGTH)
    df_loss = df_loss.append({'sensor_name':name, 'LR':lr,
                              'loss':final_test_loss},
                    ignore_index=True)
        # else:
        #     pass
