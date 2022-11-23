# -*- coding: utf-8 -*-
"""
Created on Wed Aug 31 22:23:05 2022

@author: mleem
"""

import pandas as pd

from utils import *

NUM_PRE = 30
WINDOWS = 30
tol = 0.01

df_sync = pd.DataFrame(columns=["folder_name", "pre_post", "time"])

# read the folder name
(exp_list, exp_name_list) = folder_path_name(
    "../bin/calibration/CHAR_1121_260/", "start", "Calibration", 1)

for (file, name) in zip(exp_list, exp_name_list):

    name_path = file + "/"
    # read folder name - size direction number
    (calib_test_list, calib_name_list) = folder_path_name(
        name_path, "end", ".txt", 0)
    # for loop - individual sensor folders
    for (test_list, name_list) in zip(calib_test_list, calib_name_list):
        # sensor number for pcb sequence
        sensor_num = str(name_list[-1])
        sensor_dir = str(name_list[3:-2])
        # internal path - individual sensor
        int_name_path = str(test_list)+"/"
        (int_list, int_name_list) = folder_path_name(int_name_path)
        # interpolation preprocessing
        for (int_path, int_name) in zip(int_list, int_name_list):
            # instron data
            if int_name.startswith("interlink") == 1:
                # instron data sync, preprocessing
                sync_time = instron_interp_preprocessing(
                    int_path, int_name_path + "instron_cycle300um_mod.csv")
                df_sync = df_sync.append({"folder_name": name_list,
                                          "pre_post": str(name),
                                          "time": sync_time},
                                          ignore_index=True)
            # sensor data
            elif int_name.startswith("pointSensor") == 1:
                # sensor numbering from 1
                num = int(sensor_num)
                # sensor data preprocessing
                # sensor_data = pd.read_csv(int_path, sep=" |,", header=1)
                raspi_interp_preprocessing(int_path, num,
                                            int_name_path +
                                            "sensor_cycle300um_mod.csv")

        # interpolation
        calib_interp(int_name_path + "instron_cycle300um_mod.csv",
                      sync_time,
                      int_name_path + "sensor_cycle300um_mod.csv",
                      int_name_path + "force_conversion_test.csv")

        # individual sensor for loop - interpolation check 
        for (int_path, int_name) in zip(int_list, int_name_list):
            # interpolation result
            if int_name.endswith("conversion_test.csv"):
                calib_result_plot(str(int_path),
                                  "260size", name_list+name)
                print("check_%s" % name_list)
