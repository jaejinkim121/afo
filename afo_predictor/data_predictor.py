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
import logging
from time import localtime
import time

import torch
import torch.nn as nn
import torch.nn.functional as F
from torchvision import transforms
from torch.utils.data import Dataset, DataLoader, random_split
import rospy
from std_msgs.msg import Float32MultiArray, Bool

from include.CNNmodel import *
from include.utils import *

# input size = required length*6
# required length만큼 읽어온다고 가정
# output size = 1*6 (2D array)
class dataPredictor:
    def __init__(self, start_time, data_buffer, model_name="CNN", model_dir="/home/srbl/catkin_ws/src/afo/afo_predictor/data/280/",
                 sensor_dir="Left", input_length=15, sensor_num=6, sensor_size="280",
                 thres_heel_strike=1, thres_toe_off=1, logging_prefix="", is_calibration=False):
        self.data = np.array(data_buffer)
        self.sensor_num = sensor_num
        self.sensor_dir = sensor_dir
        self.sensor_size = sensor_size
        self.model_name = model_name
        self.model_path = model_dir
        self.input_length = input_length
        self.is_calibration = is_calibration
        self._threshold_heel_strike = thres_heel_strike
        self._threshold_toe_off = thres_toe_off

        self._predicted_data = None
        self._is_swing = False
        self.current_sync = False

        self._heel_strike_detected = False
        self._toe_off_detected = False
        self._hs_num = 0
        self._to_num = 0
        self._hs_detected = False
        self._to_detected = False

        self.start_time = start_time

        self.logger = logging.getLogger(sensor_dir+'sole')
        self.logger.setLevel(logging.INFO)

        formatter = logging.Formatter('%(message)s')
        tm = time.localtime()
        logging_path = '/home/srbl/catkin_ws/src/afo/log/' + logging_prefix + '_' + time.strftime('%m%d%H%M', tm)
        file_handler = logging.FileHandler(logging_path + '_{}sole.log'.format(sensor_dir))
        file_handler.setFormatter(formatter)

        self.logger.addHandler(file_handler)
        self.logger_imu = None
        self.zero = False

        if sensor_dir=="Left":
            self.logger_imu = logging.getLogger('imu')
            self.logger_imu.setLevel(logging.INFO)
            file_handler_imu = logging.FileHandler(logging_path+'_imu.log')
            file_handler_imu.setFormatter(formatter)
            self.logger_imu.addHandler(file_handler_imu)
        
        self.device = torch.device('cpu')
        self.model_load()
        self.set_ros_node()

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

    def set_ros_node(self):
        if self.sensor_dir == "Left":
            self.data_imu_sub = rospy.Subscriber('/afo_sensor/imu', Float32MultiArray, self.callback_imu, queue_size=1)
            self.data_sub = rospy.Subscriber('/afo_sensor/soleSensor_left', Float32MultiArray, self.callback, queue_size=1)
            self.predicted_data_pub = rospy.Publisher('/afo_predictor/soleSensor_left_predicted', Float32MultiArray, queue_size=10)
            self.zero_sub = rospy.Subscriber('/afo_sync/zero', Bool, self.callback_zero, queue_size=1)
        else:
            self.data_sub = rospy.Subscriber('/afo_sensor/soleSensor_right', Float32MultiArray, self.callback, queue_size=1)
            self.predicted_data_pub = rospy.Publisher('/afo_predictor/soleSensor_right_predicted', Float32MultiArray, queue_size=10)
            self.sync_sub = rospy.Subscriber('/afo_sync/sync', Bool, self.callback_sync, queue_size=1)
            self.sync_pub = rospy.Publisher('/afo_predictor/sync_pred', Bool, queue_size=10);

    def callback(self, msg):
        data = msg.data
        self.data = np.vstack([self.data, data])[-self.input_length:]
        reel = time.time() - self.start_time
        self.logger.info('{}, {}, {}, {}'.format(reel, self.current_sync, self._is_swing, data))
        if not self.is_calibration:
            self.prediction()
        return
    
    def callback_sync(self, msg):
        self.current_sync = msg.data
        reel = time.time() - self.start_time
        msg_new = Bool()
        msg_new.data = self.current_sync
        self.sync_pub.publish(msg_new)

    def callback_zero(self, msg):
        self.zero = msg.data
        
    def callback_imu(self, msg):
        data = msg.data
        reel = time.time() - self.start_time
        self.logger_imu.info('{}, {}, {}, {}, {}'.format(reel, self.zero, self.current_sync, self._is_swing, data))

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

    def print_info(self):
        print('*     ' + str(self._heel_strike_detected) + ', num= ' + str(self._hs_num) + '  *** ' + str(self._toe_off_detected) + ', num= ' + str(self._to_num))
        self._heel_strike_detected = False
        self._toe_off_detected = False
    def phase_detection(self):
        print_arr = ["--------", "--------"]
        if self._is_swing:
            for data in self._predicted_data:
                if data > self._threshold_heel_strike:
                    self._is_swing = False
                    self._hs_num += 1
                    self._hs_detected = True
                    print_arr[0] = "detected"
                    break
        else:
            for data in self._predicted_data:
                if data > self._threshold_toe_off:
                    return
            self._is_swing = True
            self._to_num += 1
            self._to_detected = True
            print_arr[1] = "detected"
        if self._hs_detected or self._to_detected:
            print(self.sensor_dir + f"  {Fore.RED}HS{Style.RESET_ALL} " + print_arr[0] + " count: " + str(self._hs_num) + f"   {Fore.GREEN}TO{Style.RESET_ALL}: " + print_arr[1] + " count: " + str(self._to_num))
            self._hs_detected = False
            self._to_detected = False

if __name__ == "__main__":
    rospy.init_node('afo_predictor', anonymous=True)
    threshold_left_hs = float(rospy.get_param('/afo_predictor/lhs'))
    threshold_left_to = float(rospy.get_param('/afo_predictor/lto'))
    threshold_right_hs = float(rospy.get_param('/afo_predictor/rhs'))
    threshold_right_to = float(rospy.get_param('/afo_predictor/rto'))
    test_label = rospy.get_param('/afo_predictor/tl')
    is_calibration = rospy.get_param('/afo_predictor/ic')
    ros_rate = rospy.get_param('/rr')

    r = rospy.Rate(ros_rate)


    # sample data
    data_buffer = [
        [1.544, 2.024, 1.904, 1.792, 2.012, 1.984],
        [1.548, 2.028, 1.906, 1.792, 2.008, 1.982]
        ]
    
    start_time = time.time()
    is_calibration = is_calibration == 1

    left_predictor = dataPredictor(start_time, data_buffer, thres_heel_strike=threshold_left_hs, thres_toe_off=threshold_left_to, logging_prefix=test_label, is_calibration=is_calibration)
    right_predictor = dataPredictor(start_time, data_buffer, sensor_dir="Right", thres_heel_strike=threshold_right_hs, thres_toe_off=threshold_right_to, logging_prefix=test_label, is_calibration=is_calibration)
    
    while not rospy.is_shutdown():
        rospy.spin()
        r.sleep()
   
