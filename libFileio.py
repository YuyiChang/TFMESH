import scipy.io
import numpy as np
from datetime import date
import time
import matplotlib.pyplot as plt


def load_ngsim_data_raw(timePeriod, camId, lnId):
    fileDir = 'data/raw/'

    fileName = 'cam' + str(int(camId)) + '_lane' + str(int(lnId)) + '_rawdata_uncomb'

    if int(timePeriod) == 1:
        if int(camId) == 4567:
            fileName = fileName + '_v3.mat'
        elif int(camId) == 1234:
            fileName = fileName + '_v4.mat'
    elif int(timePeriod) == 2:
        fileName = fileName + '_05000515_v3_chang.mat'
    else:
        raise Exception('No avaliable data from time period' + str(timePeriod))

    print('Loading ' + fileDir + fileName)

    data = np.array(scipy.io.loadmat(fileDir+fileName)['data'])

    data[:,5] = data[:,5] * 0.248 / 3.2808 # convert pixel to meter
    data = data[data[:,5]<=625, :]    # truncate raw data to 625 m

    return data


def save_ngsim_data_step1(data, timePeriod, camId, lnId, note='step 1 output data by Yuyi'):
    fileDir = 'data/step1/'
    dateStamp = date.today().strftime("%Y%m%d")
    fileName = 'tp' + str(int(timePeriod)) + '_cam' + str(int(camId)) + '_lane' + str(int(lnId)) + '_step1_' + dateStamp + '.mat'

    data[:, [5, 11, 12]] = data[:, [5, 11, 12]] * 3.2808 # convert m back to ft
    dataS = {"data": data, "note": note}
    scipy.io.savemat(fileDir + fileName, dataS)
    print('Saved to ' + fileDir + fileName)

def load_ngsim_data_step1(timePeriod, camId, lnId, dateStamp):
    fileDir = 'data/step1/'
    fileName = 'tp' + str(int(timePeriod)) + '_cam' + str(int(camId)) + '_lane' + str(int(lnId)) + '_step1_' + str(int(dateStamp)) + '.mat'

    data = np.array(scipy.io.loadmat(fileDir+fileName)['data'])
    print('Load data from ' + fileDir + fileName)

    return data

def save_ngsim_data_step2(data, stopInfoAll, timePeriod, camId, lnId, note='step 1 output data by Yuyi'):
    fileDir = 'data/step2/'
    dateStamp = date.today().strftime("%Y%m%d")
    fileName = 'tp' + str(int(timePeriod)) + '_cam' + str(int(camId)) + '_lane' + str(int(lnId)) + '_step2_' + dateStamp + '.mat'

    dataS = {"data": data, "note": note, "stopInfoAll": stopInfoAll}
    scipy.io.savemat(fileDir + fileName, dataS)
    print('Saved to ' + fileDir + fileName)


def load_data(fileName, unit='pixel'):
    data = np.array(scipy.io.loadmat(fileName)['data'])
    if unit == 'pixel':
        data[:,5] = data[:,5] * 0.248 / 3.2808 # convert pixel to meter
        print('Data position converted from [px] to [m]')
    elif unit == 'meter':
        print('Data position already in [m]]')
    elif unit == 'foot':
        data[:,5] = data[:,5] / 3.2808 # convert pixel to meter
        print('Data position converted from [ft] to [m]')
    else:
        raise NameError('input unit shall be one of "pixel", "foot", "meter"')

    return data

def load_veh_length(fileName):
    data = np.array(scipy.io.loadmat(fileName)['lengthmatrix_all'])

    return data


def save_data_step1(data, fileDir, fileName):
    dateStamp = date.today().strftime("%Y%m%d")

    data[:, [5, 11, 12]] = data[:, [5, 11, 12]] * 3.2808 # convert m back to ft
    dataS = {"data": data}
    scipy.io.savemat(fileDir + fileName, dataS, do_compression=True)
    print('Saved to ' + fileDir + fileName)

def save_data_step2(data, fileDir, fileName):
    dateStamp = date.today().strftime("%Y%m%d")

    data[:, [5, 11, 12]] = data[:, [5, 11, 12]]
    dataS = {"data": data}
    scipy.io.savemat(fileDir + fileName, dataS, do_compression=True)
    print('Saved to ' + fileDir + fileName)


def save_data(data, fileDir, fileName, varName):
    stopInfo = {"varName": data}
    scipy.io.savemat(fileDir + fileName, stopInfo, do_compression=True)
    print('Saved to ' + fileDir + fileName)


def get_veh(data, vid):
    return data[data[:,0]==vid, :]

# def encode_veh(data, camArr):
#     IND_POS = 5
#     IND_FID = 1
#     IND_CAM_ID = 18
#     IND_LANE_ID = 13
    
#     vid = data[0,0]

#     fidStart = int(data[0,IND_FID])
#     fidEnd = int(data[-1,IND_FID])
#     numFrame = fidEnd - fidStart + 1

#     fidArr = np.arange(fidStart, fidEnd+1)
#     posMat = np.empty((numFrame, 4))
#     posMat[:] = np.nan
#     lnMat = posMat.copy()

#     for i, camId in enumerate(camArr):
#         indThisCam = np.where(data[:,IND_CAM_ID] == camId)[0]
#         fidThisCam = data[indThisCam, 1]

#         indInsert = np.add(fidThisCam, -fidStart).astype('int')
#         posMat[indInsert, i] = data[indThisCam, IND_POS]
#         lnMat[indInsert, i] = data[indThisCam, IND_LANE_ID]

#     return posMat, lnMat
#     # posMat = np.zeros()


def show_veh(dataVeh):
    plt.figure(figsize=[6,5])
    plt.subplot(311)
    plt.plot(dataVeh[:,1], dataVeh[:,5])
    plt.xlabel('frame ID')
    plt.ylabel('position (m)')
    plt.subplot(312)
    plt.plot(dataVeh[:,1], dataVeh[:,11])
    plt.xlabel('frame ID')
    plt.ylabel('speed (m/s)')
    plt.subplot(313)
    plt.plot(dataVeh[:,1], dataVeh[:,12])
    plt.xlabel('frame ID')
    plt.ylabel('accel (m/s^2)')


def compare_veh(dataSrc, dataDst):
    plt.figure(figsize=[15,12])
    plt.plot(dataSrc[:,1], dataSrc[:,5], 'r')
    plt.plot(dataDst[:,1], dataDst[:,5], 'b--')
    plt.xlabel('frame ID')
    plt.ylabel('position (m)')


def get_veh_length(vehLength, vid):
    isMissing = False
    ind = np.where(vehLength[:,0]==vid)[0]

    if len(ind) != 1:
        # print('Warning!')
        isMissing = True

    return vehLength[ind,1], isMissing