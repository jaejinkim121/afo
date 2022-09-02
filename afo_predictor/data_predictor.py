#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Sun Aug 21 01:38:40 2022

@author: minhee
"""
import os
import sys
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from tqdm import tqdm
from colorama import Fore, Style

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
class dataPredictor:
    def __init__(self, data_buffer, model_name="CNN", model_dir="/home/srbl/catkin_ws/src/afo/afo_predictor/data/280/",
                 sensor_dir="Left", input_length=15, sensor_num=6,
                 thres_heel_strike=1, thres_toe_off=1):
        self.data = np.array(data_buffer)
        self.sensor_num = sensor_num
        self.sensor_dir = sensor_dir
        self.model_name = model_name
        self.model_path = model_dir
        self.input_length = input_length
        self.device = torch.device('cpu')
        self.set_ros_node()
        self._predicted_data = None
        self._is_swing = False
        self._threshold_heel_strike = thres_heel_strike
        self._threshold_toe_off = thres_toe_off

    def set_ros_node(self):
        if self.sensor_dir == "Left":
            self.data_sub = rospy.Subscriber('/afo_sensor/soleSensor_left', Float32MultiArray, self.callback)
            self.predicted_data_pub = rospy.Publisher('/afo_predictor/soleSensor_left_predicted', Float32MultiArray, queue_size=100)
        else:
            self.data_sub = rospy.Subscriber('/afo_sensor/soleSensor_right', Float32MultiArray, self.callback)
            self.predicted_data_pub = rospy.Publisher('/afo_predictor/soleSensor_right_predicted', Float32MultiArray, queue_size=100)

    def callback(self, msg):
        data = msg.data
        self.data = np.vstack([self.data, data])
        self.prediction()
        return

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
        if self.model_name == "CNN":
            output = self.prediction_by_CNN()
        elif self.model_name == "RNN":
            output = self.prediction_by_RNN()
        else:
            pass
        #############################################################
        #############################################################
        # 재진 추가
        msg = Float32MultiArray()
        msg.data = output[0]
        self._predicted_data = output[0]
        self.phase_detection()
        self.predicted_data_pub.publish(msg)
        #############################################################
        return output

    def prediction_by_CNN(self):
        _, sensor_name_list = folder_path_name(
            self.model_path + "CNN_model/", "include", self.sensor_dir)
        sensor_name_list = [name for name in sensor_name_list if \
                            int(name[-4]) <= self.sensor_num]
        sorted_name_list = sorted(sensor_name_list, key=lambda x: int(x[-4]),
                                  reverse=False)
        model = Conv1DNet()
        prediction = np.array([])

        for name in sorted_name_list:
            model.load_state_dict(torch.load(
                self.model_path + "CNN_model/" + name,
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

    def phase_detection(self):
        if self._is_swing:
            for data in self._predicted_data:
                print(data)
                if data > self._threshold_heel_strike:
                    self._is_swing = False
                    print(self.sensor_dir + f"  {Fore.RED}HEEEEEEEEEL STRIKE{Style.RESET_ALL}")
                    break
        else:
            for data in self._predicted_data:
                if data > self._threshold_toe_off:
                    return
            self._is_swing = True
            print(self.sensor_dir + f"  {Fore.BLUE}TOOOOOE OFF{Style.RESET_ALL}")


if __name__ == "__main__":
    rospy.init_node('afo_predictor', anonymous=True)
    r = rospy.Rate(200)

    threshold_left_hs = rospy.get_param('/afo_predictor/lhs')
    threshold_left_to = rospy.get_param('/afo_predictor/lto')
    threshold_right_hs = rospy.get_param('/afo_predictor/rhs')
    threshold_right_to = rospy.get_param('/afo_predictor/rto')
    print(type(threshold_left_hs))
    print(threshold_left_hs)
    # sample data
    data_buffer = [
        [1.544, 2.024, 1.904, 1.792, 2.012, 1.984],
        [1.548, 2.028, 1.906, 1.792, 2.008, 1.982]
        ]
    
    left_predictor = dataPredictor(data_buffer, thres_heel_strike=threshold_left_hs, thres_toe_off=threshold_left_to)
    right_predictor = dataPredictor(data_buffer, sensor_dir="Right", thres_heel_strike=threshold_right_hs, thres_toe_off=threshold_right_to)
    
    while not rospy.is_shutdown():
        rospy.spin()
        r.sleep()
    
