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

from include.model import *
from include.utils import *


class dataPredictionReplay:
    def __init__(self, data, model_name="LSTM",
                 model_dir="./data/CHAR_1010_280/",
                 sensor_dir="Left", sensor_size="280",
                 input_length=15, sensor_num=6):
        self.total_data = data
        self.sensor_num = sensor_num
        self.sensor_dir = sensor_dir
        self.sensor_size = sensor_size
        self.model_name = model_name
        self.model_path = model_dir
        self.input_length = input_length
        self.device = torch.device('cpu')
        self.model_load()
        
    def data_indexing(self, idx):
        if idx <= len(self.total_data)-2:
            self.data = self.total_data[0:idx+1][:]
        else:
            # end condition
            self.data = np.zeros((1, 6))

    def model_load(self):
        self.model = np.array([])
        for num in np.arange(self.sensor_num):
            if self.model_name == "CNN":
                model = Conv1DNet()
            elif self.model_name == "LSTM":
                model = LSTM()
            else:
                pass
            model.load_state_dict(torch.load(
                self.model_path + self.model_name + "_model/" +
                self.sensor_size + self.sensor_dir + "_" +
                str(num + 1) + ".pt",
                map_location=self.device))
            self.model = np.append(self.model, model)

    def CNNtransform(self, idx):
        if len(self.data) < self.input_length:
            extra_row = np.repeat([self.data[0]], repeats=
                                  self.input_length - len(self.data), axis=0)
            self.data = np.concatenate((extra_row, self.data), axis=0)

        x = self.data[(len(self.data) - self.input_length) : len(self.data),
                      (idx - 1)]
        x = torch.from_numpy(x)
        x = x.unsqueeze(0)
        x = torch.stack([x], dim=0).float()
        return x

    def LSTMtransform(self, idx):
        if len(self.data) < self.input_length:
            extra_row = np.repeat([self.data[0]], repeats=
                                  self.input_length - len(self.data), axis=0)
            self.data = np.concatenate((extra_row, self.data), axis=0)

        x = self.data[(len(self.data) - self.input_length) : len(self.data),
                      (idx - 1)]
        x = torch.from_numpy(x)
        x = x.unsqueeze(1)
        x = torch.stack([x], dim=0).float()
        return x

    def prediction(self, idx):
        self.data_indexing(idx)
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
                if self.model_name == "CNN":
                    x = self.CNNtransform(int(name[-4]))
                elif self.model_name == "LSTM":
                    x = self.LSTMtransform(int(name[-4]))
                else:
                    pass
                output = np.append(output, model(x))

        output = np.expand_dims(output, axis=0)

        return output


if __name__ == "__main__":

    # TRIAL NUMBER
    trial_num = 14
    # trial_num_list = [11, 12, 13, 14]
    clinical_data_path = "../afo_sensor/log/data/RH-%s/sensor/" \
        % str(trial_num).zfill(2)
    prediction_path = "../afo_sensor/log/data/RH-%s/force/" \
        % str(trial_num).zfill(2)
    try:
        if not os.path.exists(prediction_path):
            os.makedirs(prediction_path)
    except:
        pass
    # calibration path, sensor size, model_name
    calibration_path = "CHAR_1010_280"
    calibration_size = "280"
    model_name = "LSTM"
    # TEST INDEX
    test_index = "main"
    test_index_list = ["launch_test", "sensor_test", "sync", "stance"]
   
    ###########################################################################
    ###########################################################################
    for test_index in test_index_list:

        # sensor path assignment
        (sensor_path, _) = folder_path_name(clinical_data_path, "start",
                                            test_index, 1)
        left_sensor_path = [path for path in sensor_path 
                            if path.endswith("Leftsole.csv")==1]
        right_sensor_path = [path for path in sensor_path 
                            if path.endswith("Rightsole.csv")==1]
    
        # left sensor replay
        left_data, left_data_front, left_header = vout_preprocessing(
            left_sensor_path[0])
        left_predictor = dataPredictionReplay(left_data,
                                              model_name = model_name,
                                              model_dir="./data/"+calibration_path+"/",
                                              sensor_size=calibration_size)
        left_force_data = left_predictor.prediction(idx=0)
        left_idx = 1
        while len(left_predictor.total_data)-left_idx >= 1:
            left_force_data = np.append(left_force_data,
                                        left_predictor.prediction(idx=left_idx),
                                        axis=0)
            left_idx += 1
    
        # right sensor replay
        right_data, right_data_front, right_header = vout_preprocessing(
            right_sensor_path[0])
        right_predictor = dataPredictionReplay(right_data,
                                              model_name = model_name,
                                              model_dir="./data/"+calibration_path+"/",
                                              sensor_size=calibration_size,
                                              sensor_dir="Right")
        right_force_data = right_predictor.prediction(idx=0)
        right_idx = 1
        while len(right_predictor.total_data)-right_idx >= 1:
            right_force_data = np.append(right_force_data,
                                        right_predictor.prediction(idx=right_idx),
                                        axis=0)
            right_idx += 1
    
        # save sensor data csv files
        left_data_front = pd.DataFrame(left_data_front)
        left_force_data = pd.DataFrame(left_force_data)
        left_force_data = pd.concat([left_data_front, left_force_data], axis=1)
        left_force_data.columns = left_header
        left_force_data.to_csv(
            prediction_path+"%s_%s_%s_Leftsole.csv" % (
                test_index, model_name, calibration_path),
            sep=",", header=True, index=False)
    
        right_data_front = pd.DataFrame(right_data_front)
        right_force_data = pd.DataFrame(right_force_data)
        right_force_data = pd.concat([right_data_front, right_force_data], axis=1)
        right_force_data.columns = right_header
        right_force_data.to_csv(
            prediction_path+"%s_%s_%s_Rightsole.csv" % (
                test_index, model_name, calibration_path),
            sep=",", header=True, index=False)
