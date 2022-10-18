# -*- coding: utf-8 -*-
"""
Created on Thu Sep  1 16:00:11 2022

@author: minhee
"""

import os
import numpy as np
import pandas as pd
import torch

from include.model import *
from include.utils import *


class DataPredictionReplay:
    def __init__(self, data, model_name="LSTM",
                 model_dir="./bin/model/CHAR_1010_280/",
                 sensor_dir="Left", sensor_size="280",
                 input_length=15, sensor_num=6):
        self.total_data = data
        self.sensor_num = sensor_num
        self.sensor_dir = sensor_dir
        self.sensor_size = sensor_size
        self.model_name = model_name
        self.model_path = model_dir
        self.input_length = input_length
        self.device = torch.device('cuda') \
            if torch.cuda.is_available else torch.device('cpu')
        print("Using PyTorch version: {}, Device: {}".format(
            torch.__version__, self.device))
        self.model_load()
        
    def data_indexing(self, idx):
        if idx <= len(self.total_data)-1:
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
                self.model_path + self.model_name + "/" +
                self.sensor_size + self.sensor_dir + "_" +
                str(num + 1) + ".pt",
                map_location=torch.device('cpu')))
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
            "/", "include", self.sensor_dir)
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
                output = np.append(output, model(x).detach().cpu().numpy())

        output = np.expand_dims(output, axis=0)

        return output


if __name__ == "__main__":

    # TRIAL NUMBER
    trial_num_list = [11, 12, 13, 14]
    # Model name list
    # model_list = ["LSTM", "CNN"]
    model_list = ["CNN"]
    # TEST INDEX
    test_index_list = ["main_", "main2_", "main3_", "main4_",
                       "stance_", "sensor_test_"]
    # Calibration path, sensor size
    calib_dict = {"11": ["CHAR_0919_280", "280"],
                  "12": ["CHAR_0927_260", "260"],
                  "13": ["CHAR_1004_280", "280"],
                  "14": ["CHAR_1010_280", "280"]}

    ###########################################################################
    ###########################################################################
    # for loop for model list
    for model_name in model_list:
        print("START %s model prediction!" % model_name)
        # for loop for trial num
        for trial_num in trial_num_list:
            print("Current trial num: %s" % str(trial_num).zfill(2))
            # for loop for test index
            for test_index in test_index_list:
                print("Current test index: %s" % test_index)

                clinical_data_path = "./bin/sensor/RH-%s/" \
                    % str(trial_num).zfill(2)
                prediction_path = "./bin/prediction/RH-%s/%s/" \
                    % (str(trial_num).zfill(2), model_name)
                try:
                    if not os.path.exists(prediction_path):
                        os.makedirs(prediction_path)
                except:
                    pass

                # calibration path, sensor size
                calibration_path = calib_dict[str(trial_num).zfill(2)][0]
                calibration_size = calib_dict[str(trial_num).zfill(2)][1]

                # sensor path assignment
                (sensor_path, _) = folder_path_name(
                    clinical_data_path, "start", test_index, 1)
                left_sensor_path = [path for path in sensor_path 
                                    if path.endswith("Leftsole.csv")==1]
                right_sensor_path = [path for path in sensor_path 
                                    if path.endswith("Rightsole.csv")==1]

                # Exception in test index
                if len(left_sensor_path) != 1:
                    pass
                else:

                    # left sensor replay
                    left_data, left_data_front, left_header = vout_preprocessing(
                        left_sensor_path[0])
                    left_predictor = DataPredictionReplay(
                        left_data,
                        model_name = model_name,
                        model_dir="./bin/model/"+calibration_path+"/",
                        sensor_size=calibration_size)
                    left_force_data = left_predictor.prediction(idx=0)
                    left_idx = 1
                    for _ in np.arange(1, len(left_predictor.total_data)):
                        left_force_data = np.append(
                            left_force_data,
                            left_predictor.prediction(idx=left_idx),
                            axis=0)
                        left_idx += 1
                
                    # right sensor replay
                    right_data, right_data_front, right_header = vout_preprocessing(
                        right_sensor_path[0])
                    right_predictor = DataPredictionReplay(
                        right_data,
                        model_name = model_name,
                        model_dir="./bin/model/"+calibration_path+"/",
                        sensor_size=calibration_size,
                        sensor_dir="Right")
                    right_force_data = right_predictor.prediction(idx=0)
                    right_idx = 1
                    for _ in np.arange(1, len(right_predictor.total_data)):
                        right_force_data = np.append(
                            right_force_data,
                            right_predictor.prediction(idx=right_idx),
                            axis=0)
                        right_idx += 1
                
                    # save sensor data csv files
                    left_data_front = pd.DataFrame(left_data_front)
                    left_force_data = pd.DataFrame(left_force_data)
                    left_force_data = pd.concat(
                        [left_data_front, left_force_data], axis=1)
                    left_force_data.columns = left_header
                    left_force_data.to_csv(
                        prediction_path+"%sLeftsole.csv" % (test_index),
                        sep=",", header=True, index=False)
                
                    right_data_front = pd.DataFrame(right_data_front)
                    right_force_data = pd.DataFrame(right_force_data)
                    right_force_data = pd.concat(
                        [right_data_front, right_force_data], axis=1)
                    right_force_data.columns = right_header
                    right_force_data.to_csv(
                        prediction_path+"%sRightsole.csv" % (test_index),
                        sep=",", header=True, index=False)
