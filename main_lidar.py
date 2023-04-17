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

## data folder
# OR, specify input data folder
srcDataDir = 'data/lidar_final/' # NGSIM

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
        
    data = f.load_data(srcDataDir + target_file_name, unit='foot')

    sz = np.shape(data)

    if sz[1] < 23:
        tmp = np.zeros((sz[0], 23-sz[1]))
        print(np.shape(tmp))
        # data[:,22] = np.zeros((sz[0],))
        data = np.hstack((data, tmp))


    print(srcDataDir + target_file_name)
    # get vehicle ID list
    vidList = np.unique(data[:,0])
    print('There are ' + str(len(vidList)) + ' vehicles in this dataset.')


    ####### step 1
    startTime = time.time() # time run time
    dataStack = Parallel(n_jobs=8)(delayed(veh_motion_est)(data, vid, config_file_path=config_file_path) for vid in tqdm(vidList))
    dataX = np.concatenate(dataStack)
    print('\nruntime = {:.2f}'.format(time.time() - startTime))

    f.save_data_step1(dataX, srcDataDir + 'output/', target_file_name)

    ##################################################################
    ####### step 2
    data = dataX.copy()             
    vidList = np.unique(data[:,0])  # get vehicle ID list
    startTime = time.time()         # time run time

    ##################################################################
    ##### step 2
    dataS = data.copy()             # make a copy for stop processing
    stopInfoAll = np.zeros((1,5))   # init stop info matrix
    for vid in tqdm(vidList):
        ind = dataS[:,0]==vid
        dataVehS, isStopped, stopInfo = s2.ns_and_s_handle(dataS[ind, :])
        if isStopped:
            dataS[ind, :] = dataVehS
            stopInfoAll = np.vstack((stopInfoAll, stopInfo))
            
    stopInfoAll = stopInfoAll[1:,:]

    print('\nruntime = {:.2f}'.format(time.time() - startTime))

    f.save_data_step2(dataS, srcDataDir + 'output/', target_file_name.replace('.mat','') +  '_op.mat')
    f.save_data(stopInfoAll, srcDataDir + 'output/', target_file_name.replace('.mat','') + '_op_stop_info.mat', "stopInfo")