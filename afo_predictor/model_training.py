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


from include.model import *
from include.utils import *


def plot_history(history, name):
    fig, ax = plt.subplots(1, 2, figsize=(13, 4))
    fig.suptitle("%s" % str(name))
    ax1 = plt.subplot(1, 2, 1)
    ax1.set_title("Training and Validation Loss")
    ax1.plot(history['train_loss'], label="train_loss")
    ax1.plot(history['test_loss'], label="test_loss")
    ax1.set_xlabel("iterations")
    ax1.set_ylabel("Loss")
    ax1.legend()
    ax2 = plt.subplot(1, 2, 2)
    ax2.set_title("Learning Rate")
    ax2.plot(history['lr'], label="learning rate")
    ax2.set_xlabel("iterations")
    ax2.set_ylabel("LR")
    plt.show()


def LSTMtraining(data_dir, DEVICE, model_dir=None,
                BATCH_SIZE=128, EPOCHS=35, TRAIN_SIZE=0.85, LR=0.00001,
                N_WARMUP_STEPS=5, DECAY_RATE=0.98, INPUT_LENGTH=15):
    dataset = CustomLSTMDataset(input_length=INPUT_LENGTH, data_dir=data_dir)

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

    model = LSTM().to(DEVICE)
    optimizer = torch.optim.Adam(model.parameters(), LR)
    scheduler = ScheduleOptim(optimizer, N_WARMUP_STEPS, DECAY_RATE)
    # criterion = nn.HuberLoss()
    criterion = RMSELoss()

    patience = 0
    best_loss = 1000
    history = {'train_loss':[], 'test_loss':[], 'lr':[]}

    for epoch in range(1, EPOCHS + 1):
        train_loss = train(model, train_dataloader, scheduler,
                           epoch, criterion, DEVICE)
        test_loss = evaluate(model, test_dataloader, criterion, DEVICE)
        lr = scheduler.get_lr()
        print("\n[EPOCH: {:2d}], \tModel: LSTM, \tLR: {:8.5f}, ".format(
            epoch, lr) \
            + "\tTrain Loss: {:8.3f}, \tTest Loss: {:8.3f} \n".format(
                train_loss, test_loss))

        history['train_loss'].append(train_loss)
        history['test_loss'].append(test_loss)
        history['lr'].append(lr)

        # Early stopping
        if test_loss < best_loss:
            best_loss = test_loss
            patience = 0
        else:
            patience += 1
            if patience >= 5:
                break

    # LSTM model save
    if model_dir is not None:
        torch.save(model.state_dict(), model_dir)

    return test_loss, history


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
    history = {'train_loss':[], 'test_loss':[], 'lr':[]}

    for epoch in range(1, EPOCHS + 1):
        train_loss = train(model, train_dataloader, scheduler,
                           epoch, criterion, DEVICE)
        test_loss = evaluate(model, test_dataloader, criterion, DEVICE)
        lr = scheduler.get_lr()
        print("\n[EPOCH: {:2d}], \tModel: Conv1DNet, \tLR: {:8.5f}, ".format(
            epoch, lr) \
            + "\tTrain Loss: {:8.3f}, \tTest Loss: {:8.3f} \n".format(
                train_loss, test_loss))

        history['train_loss'].append(train_loss)
        history['test_loss'].append(test_loss)
        history['lr'].append(lr)

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

    return test_loss, history

# hyperparameter settings
BATCH_SIZE = 32
EPOCHS = 150
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
calib_data_path = "C:/Projects/DL/pytorch112/afo/afo_predictor/bin/"\
    + "calibration/CHAR_1121_260/Calibration/"
model_path = "C:/Projects/DL/pytorch112/afo/afo_predictor/bin/"\
    + "model/CHAR_1121_260/LSTM_optim/"

try:
    if not os.path.exists(model_path):
        os.makedirs(model_path)
except:
    pass

# for loop for 12 sensor training
_, sensor_name_list = folder_path_name(calib_data_path)

df_loss2 = pd.DataFrame(columns=["Date", "sensor_name", "LR", "loss"])
# for LR in [100, 10, 1, 0.1, 0.01, 0.001, 0.0001, 0.00001]:

for n, name in enumerate(sensor_name_list):
    
    LR_list = [0.001,0.001,0.001,0.001,0.001,0.001,
                0.001,0.001,0.001,0.001,0.001,0.001]
    LR = LR_list[n]
    
    for lr in [LR-LR/2, LR-LR/4, LR-LR/5, LR-LR/10, LR, LR+LR/10, LR+LR/5,
               LR+LR, LR+LR*2, LR+LR*4, LR+LR*8, LR+LR*16, LR+LR*32]:
    # for lr in [LR-LR/5, LR-LR/10, LR, LR+LR/10, LR+LR/5]:
        # if (name.endswith("Right_2") == 1) | (name.endswith("Right_4") == 1):
        # lr = LR
        print("START LSTM MODEL TRAINING!!! %s" % name)
        print("LR: %f" % lr)
        data_dir = calib_data_path + name + "/force_conversion_test.csv"
        model_dir = model_path + name + "LR%s.pt" % lr
        final_test_loss, history = LSTMtraining(data_dir, DEVICE,
                                      model_dir=model_dir,
                                      BATCH_SIZE=BATCH_SIZE, EPOCHS=EPOCHS,
                                      LR=lr, N_WARMUP_STEPS=N_WARMUP_STEPS,
                                      DECAY_RATE=DECAY_RATE,
                                      INPUT_LENGTH=INPUT_LENGTH)
        df_loss2 = df_loss2.append({'Date': "221121_260_LSTM", 'sensor_name':name,
                                  'LR':lr, 'loss':final_test_loss},
                        ignore_index=True)
        plot_history(history, "221121_260_LSTM_LR_%s_" % str(lr) + name)
        # else:
        #     pass


# # define calib data path and model path
# calib_data_path = "C:/Projects/DL/pytorch112/afo/afo_predictor/bin/"\
#     + "calibration/CHAR_0927_260/Calibration/"
# model_path = "C:/Projects/DL/pytorch112/afo/afo_predictor/bin/"\
#     + "model/CHAR_0927_260/CNN_optim/"
# try:
#     if not os.path.exists(model_path):
#         os.makedirs(model_path)
# except:
#     pass

# # for loop for 12 sensor training
# _, sensor_name_list = folder_path_name(calib_data_path)

# df_loss2 = pd.DataFrame(columns=["Date", "sensor_name", "LR", "loss"])
# # for LR in [100, 10, 1, 0.1, 0.01, 0.001, 0.0001, 0.00001]:

# for n, name in enumerate(sensor_name_list):
    
#     LR_list = [0.001,0.001,0.001,0.001,0.001,0.001,
#                 0.001,0.001,0.001,0.001,0.001,0.001]
#     LR = LR_list[n]
    
#     for lr in [LR-LR/5, LR-LR/10, LR, LR+LR/10, LR+LR/5]:
#     # for lr in [LR-LR/5, LR-LR/10, LR, LR+LR/10, LR+LR/5]:
#     # if (name.endswith("Right_4") == 1) | (name.endswith("Left_3") == 1):
#     # if (name.endswith("Left_1") == 1):
#         # lr = LR
#         print("START 1D CNN MODEL TRAINING!!! %s" % name)
#         print("LR: %f" % lr)
#         data_dir = calib_data_path + name + "/force_conversion_test.csv"
#         model_dir = model_path + name + "LR%s.pt" % lr
#         final_test_loss, history = CNNtraining(data_dir, DEVICE,
#                                       model_dir=model_dir,
#                                       BATCH_SIZE=BATCH_SIZE, EPOCHS=EPOCHS,
#                                       LR=lr, N_WARMUP_STEPS=N_WARMUP_STEPS,
#                                       DECAY_RATE=DECAY_RATE,
#                                       INPUT_LENGTH=INPUT_LENGTH)
#         df_loss2 = df_loss2.append({'Date': "220927_260_CNN", 'sensor_name':name,
#                                   'LR':lr, 'loss':final_test_loss},
#                         ignore_index=True)
#         plot_history(history, "220927_CNN_LR_%s_" % str(lr) + name)
# # else:
# #     pass

# # define calib data path and model path
# calib_data_path = "C:/Projects/DL/pytorch112/afo/afo_predictor/bin/"\
#     + "calibration/CHAR_1031_280/Calibration/"
# model_path = "C:/Projects/DL/pytorch112/afo/afo_predictor/bin/"\
#     + "model/CHAR_1031_280/LSTM_optim/"

# try:
#     if not os.path.exists(model_path):
#         os.makedirs(model_path)
# except:
#     pass

# # for loop for 12 sensor training
# _, sensor_name_list = folder_path_name(calib_data_path)

# df_loss3 = pd.DataFrame(columns=["Date", "sensor_name", "LR", "loss"])
# # for LR in [100, 10, 1, 0.1, 0.01, 0.001, 0.0001, 0.00001]:

# for n, name in enumerate(sensor_name_list):
    
#     LR_list = [0.001,0.001,0.001,0.001,0.001,0.001]
#     LR = LR_list[n]
    
#     for lr in [LR-LR/2, LR-LR/4, LR+LR, LR+LR*2, LR+LR*4, LR+LR*8, LR+LR*16, LR+LR*32]:
#     # for lr in [LR-LR/5, LR-LR/10, LR, LR+LR/10, LR+LR/5]:
#     # if (name.endswith("Left_1") == 1):
#         # lr = LR
#         print("START LSTM MODEL TRAINING!!! %s" % name)
#         print("LR: %f" % lr)
#         data_dir = calib_data_path + name + "/force_conversion_test.csv"
#         model_dir = model_path + name + "LR%s.pt" % lr
#         final_test_loss, history = LSTMtraining(data_dir, DEVICE,
#                                       model_dir=model_dir,
#                                       BATCH_SIZE=BATCH_SIZE, EPOCHS=EPOCHS,
#                                       LR=lr, N_WARMUP_STEPS=N_WARMUP_STEPS,
#                                       DECAY_RATE=DECAY_RATE,
#                                       INPUT_LENGTH=INPUT_LENGTH)
#         df_loss3 = df_loss3.append({'Date': "221031_280_LSTM", 'sensor_name':name,
#                                   'LR':lr, 'loss':final_test_loss},
#                         ignore_index=True)
#         plot_history(history, "221031_280_LSTM_LR_%s_" % str(lr) + name)
#         # else:
#         #     pass
