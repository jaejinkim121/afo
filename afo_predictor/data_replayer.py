import os
import sys
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import logging
from time import localtime
import time

import torch
import torch.nn as nn
import torch.nn.functional as F
from torchvision import transforms
from torch.utils.data import Dataset, DataLoader, random_split

from include.CNNmodel import *
from include.utils import *

class Replayer:
    def __init__(self, start_time, data_buffer, model_name="CNN",
                 model_dir="./data/280/",
                 sensor_dir="Left", input_length=15, sensor_num=6,
                 sensor_size="280",
                 thres_heel_strike=1, thres_toe_off=1
                 ):

        self.data = np.array(data_buffer)
        self.sensor_num = sensor_num
        self.sensor_dir = sensor_dir
        self.sensor_size = sensor_size
        self.model_name = model_name
        self.model_path = model_dir
        self.input_length = input_length
        self._threshold_heel_strike = thres_heel_strike
        self._threshold_toe_off = thres_toe_off

        self._predicted_data = None
        self._is_swing = False

        self._heel_strike_detected = False
        self._toe_off_detected = False
        self._hs_detected = False
        self._to_detected = False

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

        x = self.data[(len(self.data) - self.input_length): len(self.data),
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
        #############################################################
        #############################################################
        # 재진 추가
        self._predicted_data = output[0]
        self.phase_detection()
        #############################################################
        return output

    def phase_detection(self):
        if self._is_swing:
            for data in self._predicted_data:
                if data > self._threshold_heel_strike:
                    self._is_swing = False
                    self._hs_detected = True
                    break
        else:
            for data in self._predicted_data:
                if data > self._threshold_toe_off:
                    return
            self._is_swing = True
            self._to_detected = True

        if self._hs_detected or self._to_detected:
            self._hs_detected = False
            self._to_detected = False

    def load_data(self, data):
        self.data = np.vstack([self.data, data])[-self.input_length:]
        self.prediction()


def main():
    ...
