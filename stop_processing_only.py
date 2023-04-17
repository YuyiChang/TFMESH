# %%
import numpy as np
import matplotlib.pyplot as plt
import time
import warnings
from tqdm import tqdm
from joblib import Parallel, delayed
import multiprocessing
import os

import libStep1 as s1
import libStep2 as s2
import libFileio as f
import importlib

import glob

import pandas as pd

# data folder
# OR, specify input data folder
srcDataDir = 'data/lidar_final_stop_proc_only/' # NGSIM

# load configuration file
config_file_path = 'config/lidar.ini'

print(srcDataDir)

num_cores = multiprocessing.cpu_count()

# create folder under srcDataDir for data output
try:
    os.mkdir(srcDataDir + 'output')
except FileExistsError:
    pass

def veh_motion_est(data, vid, config_file_path=None):
    # get vehicle
    dataVeh = f.get_veh(data, vid)
    # est motion
    dataTemp = s1.combine_cam_motion_est_ud(dataVeh, config_file_path=config_file_path)
    return dataTemp #, np.shape(dataTemp)[0]


file_list = glob.glob(srcDataDir + '*.mat')
print("Found " + str(len(file_list)) + " data files under " + srcDataDir + "\n")


# %%

for file in file_list:
    print("\n----------------------------------------------------------------------")
    print("Working on " + file + "...")

    # load all cam data
    if os.name == 'nt':
        target_file_name = file.split('\\')[-1]
    else:
        target_file_name = file.split('/')[-1]

    ##
    data = f.load_data(srcDataDir + target_file_name, unit='meter')
    vidList = np.unique(data[:,0])  # get vehicle ID list

    stop_info = pd.read_csv(srcDataDir + 'stop_info.csv').to_numpy()
    # print(stop_info)

    vid_stopped = stop_info[:, 0]
    print(vid_stopped)
    for i, vid in enumerate(vid_stopped):
        ind = data[:,0] == vid
        data_veh = data[ind, :]

        veh_stop_info = stop_info[i,:].reshape(1, -1) - 1
        # print(veh_stop_info)

        data[ind, :] = s2.spline_near_stop(veh_stop_info, data_veh)

f.save_data_step2(data, srcDataDir + 'output/', target_file_name.replace('.mat','') +  '_op.mat')

# %%
plt.figure()
fig, ax = plt.subplots(2, 1, sharex=True)
for vid in vidList:
    ind = data[:,0]==vid
    if vid < 2000: # near lane
        i = 0
    else:
        i = 1
    ax[i].plot(data[ind, 1], data[ind, 5], 'b')


# num_stopped_veh = np.shape(stopInfoAll)[0]
# for i in range(num_stopped_veh):
#     vid = stopInfoAll[i, 0]
#     ind = data[:,0]==vid
#     data_veh = data[ind, :]
#     if vid < 2000: i = 0 # near lane
#     else: i = 1

#     idx_1, idx_2 = int(stopInfoAll[i, 2]), int(stopInfoAll[i, 3])
#     idx_1p, idx_2p = int(stopInfoAll[i, 1]), int(stopInfoAll[i, 4])
#     # print(idx_1, idx_2)
#     ax[i].plot(data_veh[idx_1, 1], data_veh[idx_1, 5], 'r>')
#     ax[i].plot(data_veh[idx_2, 1], data_veh[idx_2, 5], 'r<')
