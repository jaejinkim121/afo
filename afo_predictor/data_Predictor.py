# -*- coding: utf-8 -*-
"""
Created on Sun Aug 21 01:38:40 2022

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

import rospy
from std_msgs.msg import Float32MultiArray

from include.CNNmodel import *
from include.utils import *

# input size = required length*6
# required length만큼 읽어온다고 가정
# output size = 1*6 (2D array)
class data_Predictor():
    def __init__(self, data_buffer, model_name="CNN", model_dir="./data/RH10/",
                 sensor_dir="Left", input_length=15, sensor_num=6):
        self.data = np.array(data_buffer)
        self.sensor_num = sensor_num
        self.sensor_dir = sensor_dir
        self.model_name = model_name
        self.model_path = model_dir
        self.input_length = input_length
        self.device = torch.device('cpu')

    def transform(self, idx):
        if len(self.data) < self.input_length:
            # Buffer에 데이터가 없어서 required length만큼 읽어오지 못했을 경우
            # 0열을 부족한만큼 복사
            extra_row = np.repeat([self.data[0]], repeats=
                                  self.input_length - len(self.data), axis=0)
            self.data = np.concatenate((extra_row, self.data), axis=0)

        x = self.data[:, (idx - 1)]
        x = torch.from_numpy(x)
        x = x.unsqueeze(0)
        x = torch.stack([x], dim=0).float()
        return x

    def prediction(self):
        if self.model_name == "CNN":
            output = self.prediction_by_CNN()
        elif self.model_name == "RNN":
            output = self.prediction_by_RNN()
        else:
            pass
        #############################################################
        #############################################################
        # 재진 추가
        #############################################################
        return output

    def prediction_by_CNN(self):
        self.model_path += "CNN_model/"
        _, sensor_name_list = folder_path_name(
            self.model_path, "include", self.sensor_dir)
        sensor_name_list = [name for name in sensor_name_list if \
                            int(name[-4]) <= self.sensor_num]
        sorted_name_list = sorted(sensor_name_list, key=lambda x: int(x[-4]),
                                  reverse=False)
        model = Conv1DNet()
        prediction = np.array([])

        for name in sorted_name_list:
            model.load_state_dict(torch.load(self.model_path + name,
                                             map_location=self.device))
            model = model.to(self.device)
            model.eval()

            with torch.no_grad():
                x = self.transform(int(name[-4]))
                x = x.to(self.device)
                prediction = np.append(prediction, model(x))

        prediction = np.expand_dims(prediction, axis=0)
        return prediction

    def prediction_by_RNN(self):
        pass

if __name__ == "__main__":
    # sample data
    data_buffer = [
        [1.544, 2.024, 1.904, 1.792, 2.012, 1.984],
        [1.548, 2.028, 1.906, 1.792, 2.008, 1.982]
        ]
    real_time_predictions = data_Predictor(data_buffer).prediction()
