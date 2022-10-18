# -*- coding: utf-8 -*-
"""
Created on Tue Aug 23 00:35:03 2022

@author: minhee
"""

import glob
import os
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

def folder_path_name(path, option=None, char=None, T_F=None):

    folder_name_path = str(path)
    folder_path = folder_name_path + "*"

    file_list = glob.glob(folder_path)
    file_name_list = os.listdir(folder_name_path)

    if option == "start":
        exp_list = [file for (file, name) in zip(file_list, file_name_list)
                    if name.startswith(str(char)) == int(T_F)]
        exp_name_list = [name for (file, name) in
                         zip(file_list, file_name_list)
                         if name.startswith(str(char)) == int(T_F)]

    elif option == "end":
        exp_list = [file for (file, name) in zip(file_list, file_name_list)
                    if name.endswith(str(char)) == int(T_F)]
        exp_name_list = [name for (file, name)
                         in zip(file_list, file_name_list)
                         if name.endswith(str(char)) == int(T_F)]
        
    elif option == "include":
        exp_list = [file for (file, name) in zip(file_list, file_name_list)
                    if str(char) in name]
        exp_name_list = [name for (file, name)
                         in zip(file_list, file_name_list)
                         if str(char) in name]

    else:
        exp_list = [file for (file, name) in zip(file_list, file_name_list)]
        exp_name_list = [name for (file, name)
                         in zip(file_list, file_name_list)]

    exp_list = [file.replace('\\', '/') for file in exp_list]

    return exp_list, exp_name_list


def instron_interp_preprocessing(instron_path, save_path):

    instron_csv = instron_path
    instron_data = pd.read_csv(instron_csv, header=0)
    instron_data = pd.DataFrame.drop(instron_data, index=0, axis=0)
    instron_data.reset_index(drop=True, inplace=True)
    instron_data = instron_data.astype(float)
    ini_vol = instron_data.at[0, 'voltage_1']
    sync = np.where(abs(instron_data["voltage_1"] - ini_vol) >= 0.03)
    sync_index = sync[0][0]
    sync_time = instron_data.at[sync_index, 'Time']
    instron_data.to_csv(str(save_path), sep=",", index=False)

    return sync_time


def raspi_interp_preprocessing(raspi_path, sensor_num, save_path):

    sensor_csv = raspi_path
    sensor_data = pd.read_csv(sensor_csv, sep=" |,", header=1)
    # sensor_data = sensor_data.astype(float)
    sensor_data_ = sensor_data.iloc[:, [2, 4, int(sensor_num)+5]]

    sensor_data = sensor_data_
    sensor_data.columns = ['sync', 'time', 'vout']
    ini_sync = sensor_data.at[0, 'sync']
    sync = np.where(abs(sensor_data["sync"] - ini_sync) >= 1)
    sync_index = sync[0]
    sensor_data = sensor_data.iloc[sync_index, :]
    sensor_data.reset_index(inplace=True, drop=True)
    print(sensor_data)
    ini_time = sensor_data.at[0, 'time']
    sensor_index = list(sensor_data.index)
    del_time = []
    for j in sensor_index:
        if j == 0:
            del_time = np.append(del_time, sensor_data.at[j, 'time']-ini_time)
        else:
            del_time = np.append(del_time,
                                 sensor_data.at[j, 'time']
                                 - sensor_data.at[j-1, 'time'])
    d_time = pd.DataFrame(del_time, columns=["flag"])
    d_time = d_time[d_time["flag"] > 0.003]
    sensor_time_index = list(d_time.index)[0]
    sensor_data = sensor_data[sensor_data.index >= sensor_time_index]
    sensor_data["time"] = sensor_data["time"] - ini_time

    # only for CHAR_0927_260
    # sensor_data["vout"] = sensor_data["vout"] * (0.625/0.5)
    sensor_data.to_csv(str(save_path), sep=",", index=False)


def calib_interp(instron_path, instron_sync_time, sensor_path, save_path):

    instron_data = pd.read_csv(str(instron_path), header=0)
    instron_data = instron_data.astype(float)
    instron_synced = instron_data[(abs(instron_data["voltage_1"] -
                                       instron_data.at[0, "voltage_1"]) > 3.0)
                                  & (instron_data["Time"]
                                     >= instron_sync_time)]
    instron_synced["Time"] = instron_synced["Time"] - instron_sync_time
    instron_synced.reset_index(drop=True, inplace=True)

    sensor_data = pd.read_csv(str(sensor_path), header=0)
    sensor_data = sensor_data.astype(float)
    sensor_synced = sensor_data[abs(sensor_data["sync"] -
                                    sensor_data.at[0, "sync"]) == 0]
    sensor_synced.reset_index(drop=True, inplace=True)

    vout_synced = pd.DataFrame(np.interp(instron_synced["Time"],
                                         sensor_synced["time"],
                                         sensor_synced["vout"]),
                                columns=["vout"])
    calib_synced = pd.concat([instron_synced["Time"],
                              vout_synced, instron_synced["Force"]], axis=1)
    calib_synced.columns = ["time", "vout", "force"]
    calib_synced.to_csv(str(save_path), sep=",", index=False)


def calib_result_plot(path, RH_num, folder_name):

    data = pd.read_csv(str(path), delimiter=",", header=0)

    plt.figure(figsize=(6, 8))
    plt.plot(data.loc[:, "time"], data.loc[:, "vout"],
             label='%s_%s' % (str(RH_num), str(folder_name)))
    plt.title("time vs vout")
    plt.xlabel("Time [s]")
    plt.ylabel("vout [V]")
    plt.legend(loc='best')
    plt.tight_layout()
    plt.show()

    plt.figure(figsize=(6, 8))
    plt.plot(data.loc[:, "time"], data.loc[:, "force"],
             label='%s_%s' % (str(RH_num), str(folder_name)))
    plt.title("time vs force")
    plt.xlabel("Time [s]")
    plt.ylabel("Force [N]")
    plt.legend(loc='best')
    plt.tight_layout()
    plt.show()

    plt.figure(figsize=(6, 8))
    plt.plot(data.loc[:, "vout"], data.loc[:, "force"],
             label='%s_%s' % (str(RH_num), str(folder_name)))
    plt.title("vout vs force")
    plt.ylabel("Force [N]")
    plt.xlabel("vout [V]")
    plt.legend(loc='best')
    plt.tight_layout()
    plt.show()


def replace_multiple_text(text, dic={",": " ", "  ": " "}):
    for i, j in dic.items():
        text = text.replace(i, j)
    return text


def vout_preprocessing(path):

    sensor_csv = path

    # RH-11 exception
    if "RH-11" in sensor_csv:
        dict_ = {",": " ", "  ": " ", "(": "", ")": "",
                 "False": "0", "True": "1"}
    else:
        dict_ = {",": " ", "  ": " "}

    # Multiple delimiter
    with open(sensor_csv) as data:
        conv_data = [replace_multiple_text(line, dict_) for line in data]

    sensor_vout_header = ["swing_phase", "sync", "time",
                      "v1" ,"v2", "v3", "v4", "v5", "v6"]
    sensor_force_header = ["swing_phase", "sync", "time",
                      "f1" ,"f2", "f3", "f4", "f5", "f6"]

    # RH-11 exception data loading
    if "RH-11" in sensor_csv:
        data_pre = np.loadtxt(conv_data, delimiter=" ")
        sensor_vout_header_pre = ["time", "sync", "swing_phase",
                                  "v1" ,"v2", "v3", "v4", "v5", "v6"]
        data_1 = pd.DataFrame(data_pre, columns=sensor_vout_header_pre)
        data_ = data_1[sensor_vout_header]
        sensor_data = np.array(data_)
    else:
        sensor_data = np.loadtxt(conv_data, delimiter=" ", skiprows=1)

    data_buffer = sensor_data[:,3:]
    data_front = sensor_data[:,:3]
    return data_buffer, data_front, sensor_force_header
