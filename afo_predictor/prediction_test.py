# -*- coding: utf-8 -*-
"""
Created on Thu Sep  1 16:00:11 2022

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

def data_load(idx=0, path="./data/280/Test/test_data_left.csv"):
    data = np.loadtxt(path, delimiter=",")
    if idx <= len(data)-2:
        data_buffer = data[0:idx+1][:]
    else:
        data_buffer = np.zeros((1, 6))
    return data_buffer

# input size = required length*6
# required length만큼 읽어온다고 가정
# output size = 1*6 (2D array)
class dataPredictor:
    def __init__(self, data_buffer, model_name="CNN", model_dir="./data/280/",
                 sensor_dir="Left", sensor_size="280",
                 input_length=15, sensor_num=6):
        self.data = np.array(data_buffer)
        self.sensor_num = sensor_num
        self.sensor_dir = sensor_dir
        self.sensor_size = sensor_size
        self.model_name = model_name
        self.model_path = model_dir
        self.input_length = input_length
        self.device = torch.device('cpu')
        self.model_load()
                
    def model_load(self):
        self.model = np.array([])
        for num in np.arange(self.sensor_num):
            if self.model_name == "CNN":
                model = Conv1DNet()
            else:
                pass
            model.load_state_dict(torch.load(
                self.model_path + self.model_name + "_model/" +
                self.sensor_size + self.sensor_dir + "_" +
                str(num + 1) + ".pt",
                map_location=self.device))
            self.model = np.append(self.model, model)

    def transform(self, idx):
        if len(self.data) < self.input_length:
            # Buffer에 데이터가 없어서 required length만큼 읽어오지 못했을 경우
            # 0열을 부족한만큼 복사
            extra_row = np.repeat([self.data[0]], repeats=
                                  self.input_length - len(self.data), axis=0)
            self.data = np.concatenate((extra_row, self.data), axis=0)

        x = self.data[(len(self.data) - self.input_length) : len(self.data),
                      (idx - 1)]
        x = torch.from_numpy(x)
        x = x.unsqueeze(0)
        x = torch.stack([x], dim=0).float()
        return x

    def prediction(self):
        _, sensor_name_list = folder_path_name(
            self.model_path + self.model_name +
            "_model/", "include", self.sensor_dir)
        sensor_name_list = [name for name in sensor_name_list if \
                            int(name[-4]) <= self.sensor_num]
        sorted_name_list = sorted(sensor_name_list, key=lambda x: int(x[-4]),
                                  reverse=False)
        output = np.array([])

        for name in sorted_name_list:

            model = self.model[int(name[-4]) - 1]
            model.eval()
            with torch.no_grad():
                x = self.transform(int(name[-4]))
                output = np.append(output, model(x))

        output = np.expand_dims(output, axis=0)

        return output


if __name__ == "__main__":

    idx = 0
    data_buffer = data_load(idx)
    left_predictor = dataPredictor(data_buffer)
    right_predictor = dataPredictor(data_buffer, sensor_dir="Right")
    while data_buffer.all() != 0.0:

        # print(left_predictor.prediction())
        print(right_predictor.prediction())
        # idx += 1
        # data_buffer = data_load(idx)
