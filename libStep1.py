import numpy as np
import matplotlib.pyplot as plt
from filterpy.kalman import KalmanFilter
from filterpy.common import Q_discrete_white_noise
from datetime import date
import warnings

import numba
from numba import jit

import time


def ignore_select_vid(data, vidList):
    ind = np.full((len(data), 1), True)
    for vid in vidList:
        ind = (data[:,0] != vid) & ind
    return ind


def init_ca(dt, std):
    ca = KalmanFilter(dim_x=3, dim_z=1)
    ca.x = np.array([0., 0., 0.])
    ca.P *= 1
    ca.R *= std
    ca.Q = Q_discrete_white_noise(dim=3, dt=dt, var=2)
    ca.F = np.array([[1, dt, 0.5*dt*dt],
                     [0, 1,         dt],
                     [0, 0,          1]])
    ca.H = np.array([[1., 0, 0]])
    return ca


def est_init_v(frame, pos, dt):
    z = np.polyfit(dt*frame[0:9], pos[0:9], 1)
    return z[0]


# measurement noise model
def measurement_noise_model(pos):
    h = 152 # camera floor height
    r = 1.5 # typical car height
    M = 1435 # mid-point where proj error is min
    d = 70.5 # obs point to road

    D = np.abs(pos - M)
    a = np.sqrt(d**2 + D**2)
    b = a*r / (h - r)
    x = D*b/a/3 + 0.5

    return x


# reorganize time-space data
def reorganize_uncombine_pos(dataVeh):

    initFrame = np.amin(dataVeh[:, 1])
    lastFrame = np.amax(dataVeh[:, 1])

    numFrame = int(lastFrame - initFrame + 1)

    camIdArr = np.unique(dataVeh[:, 18])

    orgPos = np.empty((int(numFrame), len(camIdArr)))
    orgPos.fill(np.nan)

    for i, camId in enumerate(camIdArr):
        fidSel = dataVeh[dataVeh[:,18]==camId,1]
        indS = int(fidSel[0]-initFrame)
        indE = indS + len(fidSel)

        orgPos[indS:indE,i] = dataVeh[dataVeh[:,18]==camId,5]

    isDetection = np.logical_not(np.isnan(orgPos))

    return orgPos, isDetection


def reset_ax_lim(dataVeh, lb, ub):
    ind = (dataVeh[:,5] > lb) & (dataVeh[:,5] < ub)
    fidArrSel = dataVeh[ind, 1]

#     print(fidArrSel[0])
#     print(fidArrSel[-1])

    plt.xlim(fidArrSel[0], fidArrSel[-1])
    plt.ylim(lb, ub)

    return 1


def combine_cam_vehicle_data(data, vid):
    warnings.warn("deprecated: use combine_cam_motion_est() instead", DeprecationWarning)
    dt = 0.1
    dataVeh = data[data[:,0]==vid, :]
    camIdArr = np.unique(dataVeh[:, 18])
    orgPos, isDetection = reorganize_uncombine_pos(dataVeh)

    numFrame = np.shape(orgPos)[0]

    # position smoothing and v/a est
    initV = est_init_v(dataVeh[:,1], dataVeh[:,5], dt)
    cafilter = init_ca(dt, 0.02) # 0.005, 0.08
    cafilter.x = np.array([dataVeh[0,5], initV, 0])

    xs_k = []
    cov = np.zeros((numFrame, 3, 3))

    R_history = []

    for i in range(numFrame):
        cafilter.predict()
        cafilter.R = measurement_noise_model(cafilter.x[0])
        for j in range(len(camIdArr)):
            if isDetection[i, j]:
                cafilter.update(orgPos[i, j])
        xs_k.append(cafilter.x)
        cov[i,:,:] = cafilter.P

        R_history.append(cafilter.R)

    xs_k = np.array(xs_k)
    xs_rts, _, _, _ = cafilter.rts_smoother(xs_k, cov)

    fidArr = dataVeh[0, 1] + range(numFrame)

    dataTemp = np.empty((numFrame, 18))
    dataTemp.fill(0)

    dataTemp[:,0] = vid
    dataTemp[:,1] = fidArr
    dataTemp[:,5] = xs_rts[:,0]
    dataTemp[:,11] = xs_rts[:,1]
    dataTemp[:,12] = xs_rts[:,2]
    dataTemp[:,13] = get_lane_id(dataVeh)

    return dataTemp


def encode_veh(data, camArr):
    IND_POS = 5
    IND_FID = 1
    IND_CAM_ID = 18
    IND_LANE_ID = 13
    
    vid = data[0,0]

    # fidStart = int(data[0,IND_FID])
    # fidEnd = int(data[-1,IND_FID])

    fidStart = int(np.amin(data[:, IND_FID]))
    fidEnd = int(np.amax(data[:, IND_FID]))

    numFrame = fidEnd - fidStart + 1

    fidArr = np.arange(fidStart, fidEnd+1)
    posMat = np.empty((numFrame, 4))
    posMat[:] = np.nan
    lnMat = posMat.copy()

    for i, camId in enumerate(camArr):
        indThisCam = np.where(data[:,IND_CAM_ID] == camId)[0]
        fidThisCam = data[indThisCam, 1]

        indInsert = np.add(fidThisCam, -fidStart).astype('int')
        posMat[indInsert, i] = data[indThisCam, IND_POS]
        lnMat[indInsert, i] = data[indThisCam, IND_LANE_ID]

    return posMat, lnMat, fidArr


def encode_veh_ud(data):
    IND_POS = 5
    IND_FID = 1
    IND_CAM_ID = 2 # NGSIM from Lizhe
    IND_CAM_ID = 18 # for 3D LiDAR data
    IND_LANE_ID = 13
    IND_US_FLAG = 21

    camArr = np.array(())
    udArr = np.array(())
    # check id data contain both us and ds
    ind = data[:, IND_US_FLAG] == 0
    thisCam = np.unique(data[ind, IND_CAM_ID])
    numCam = len(thisCam)
    camArr = np.append(camArr, thisCam)
    udArr = np.append(udArr, np.zeros((1,len(thisCam))) )

    ind = np.logical_not(ind)
    thisCam = np.unique(data[ind, IND_CAM_ID])
    camArr = np.append(camArr, thisCam)
    numCam += len(thisCam)
    udArr = np.append(udArr, np.ones((1,len(thisCam))) )

    fidStart = int(np.amin(data[:, IND_FID]))
    fidEnd = int(np.amax(data[:, IND_FID]))

    numFrame = fidEnd - fidStart + 1

    fidArr = np.arange(fidStart, fidEnd+1)
    posMat = np.empty((numFrame, numCam))
    posMat[:] = np.nan
    lnMat = posMat.copy() # TODO

    for i, camId in enumerate(camArr):
        indThisCam = data[:,IND_CAM_ID] == camId 
        indThisUd = data[:,IND_US_FLAG] == udArr[i] 

        indThisCam = np.logical_and(indThisCam, indThisUd)

        fidThisCam = data[indThisCam, 1]

        indInsert = np.add(fidThisCam, -fidStart).astype('int')
        posMat[indInsert, i] = data[indThisCam, IND_POS]
        lnMat[indInsert, i] = data[indThisCam, IND_LANE_ID]

    # get lane indexed with frame ID
    numLaneObs = np.shape(lnMat)[1] - np.sum(np.isnan(lnMat), axis=1)
    lnArr = np.zeros((numFrame, ))


    # check if this vehicle starts with no lane assignment
    if numLaneObs[0] == 0:
        warnings.warn("Vehicle " + str(data[0,0]) + " starts with no lane assignment!", UserWarning)
        print("Vehicle " + str(data[0,0]) + " starts with no lane assignment!")
        ind = np.where(numLaneObs==1)[0][0]
        indNan = ~np.isnan(lnMat[ind,:])
        lnArr[-1] = lnMat[i,ind]

    for i in range(numFrame):
        ind = ~np.isnan(lnMat[i,:])
        numEle = len(np.unique(lnMat[i,ind]))
        if numEle == 1:
            lnArr[i] = np.unique(lnMat[i,ind])
        else:
            lnArr[i] = lnArr[i-1]
    return posMat, lnArr, fidArr, lnMat
        

# def combine_cam_motion_est(dataVeh, isUpstream=True):
#     # use this only for d/s u/s seperated data
#     dt = 0.1
#     if isUpstream:
#         camArr = np.arange(1,5)
#     else:
#         camArr = np.arange(4,8)
    
#     posMat, lnMat, fidArr = encode_veh(dataVeh, camArr)
#     numFrame = len(fidArr)
#     isDetection = np.logical_not(np.isnan(posMat))

#     initV = est_init_v(dataVeh[:,1], dataVeh[:,5], dt)
#     cafilter = init_ca(dt, 0.02) # 0.005, 0.08
#     cafilter.x = np.array([dataVeh[0,5], initV, 0])

#     xs_k = []
#     cov = np.zeros((numFrame, 3, 3))

#     R_history = []

#     for i in range(numFrame):
#         cafilter.predict()
#         cafilter.R = measurement_noise_model(cafilter.x[0])
#         for j in range(len(camArr)):
#             if isDetection[i, j]:
#                 cafilter.update(posMat[i, j])
#         xs_k.append(cafilter.x)
#         cov[i,:,:] = cafilter.P

#         R_history.append(cafilter.R)

#     xs_k = np.array(xs_k)
#     xs_rts, _, _, _ = cafilter.rts_smoother(xs_k, cov)

#     dataTemp = np.empty((numFrame, 18))
#     dataTemp.fill(0)

#     dataTemp[:,0] = dataVeh[0,0]
#     dataTemp[:,1] = fidArr
#     dataTemp[:,5] = xs_rts[:,0]
#     dataTemp[:,11] = xs_rts[:,1]
#     dataTemp[:,12] = xs_rts[:,2]
#     dataTemp[:,13] = get_lane_id(dataVeh)

#     return dataTemp


def combine_cam_motion_est_ud(dataVeh):
    dt = 0.1
    
    posMat, lnArr, fidArr, _ = encode_veh_ud(dataVeh)
    numCam = np.shape(posMat)[1]
    numFrame = len(fidArr)
    isDetection = np.logical_not(np.isnan(posMat))

    initV = est_init_v(dataVeh[:,1], dataVeh[:,5], dt)
    cafilter = init_ca(dt, 0.02) # 0.005, 0.08
    cafilter.x = np.array([dataVeh[0,5], initV, 0])

    xs_k = []
    cov = np.zeros((numFrame, 3, 3))

    R_history = []

    for i in range(numFrame):
        cafilter.predict()
        cafilter.R = measurement_noise_model(cafilter.x[0])
        for j in range(numCam):
            if isDetection[i, j]:
                cafilter.update(posMat[i, j])
        xs_k.append(cafilter.x)
        cov[i,:,:] = cafilter.P

        R_history.append(cafilter.R)

    xs_k = np.array(xs_k)
    xs_rts, _, _, _ = cafilter.rts_smoother(xs_k, cov)

    dataTemp = np.empty((numFrame, 18))
    dataTemp.fill(0)

    dataTemp[:,0] = dataVeh[0,0]
    dataTemp[:,1] = fidArr
    dataTemp[:,5] = xs_rts[:,0]
    dataTemp[:,11] = xs_rts[:,1]
    dataTemp[:,12] = xs_rts[:,2]
    dataTemp[:,13] = lnArr

    return dataTemp


def combine_cam_motion_est_ud_logging(dataVeh):
    dt = 0.1
    
    posMat, lnArr, fidArr, _ = encode_veh_ud(dataVeh)
    numCam = np.shape(posMat)[1]
    numFrame = len(fidArr)
    isDetection = np.logical_not(np.isnan(posMat))

    initV = est_init_v(dataVeh[:,1], dataVeh[:,5], dt)
    cafilter = init_ca(dt, 0.02) # 0.005, 0.08
    cafilter.x = np.array([dataVeh[0,5], initV, 0])

    xs_k = []
    cov = np.zeros((numFrame, 3, 3))
    likehood = np.zeros((numFrame, 1))

    R_history = []

    for i in range(numFrame):
        cafilter.predict()
        cafilter.R = measurement_noise_model(cafilter.x[0])
        for j in range(numCam):
            if isDetection[i, j]:
                cafilter.update(posMat[i, j])
        xs_k.append(cafilter.x)
        cov[i,:,:] = cafilter.P
        likehood[i] = cafilter.likelihood

        R_history.append(cafilter.R)

    xs_k = np.array(xs_k)
    xs_rts, covRts, _, _ = cafilter.rts_smoother(xs_k, cov)

    dataTemp = np.empty((numFrame, 18))
    dataTemp.fill(0)

    dataTemp[:,0] = dataVeh[0,0]
    dataTemp[:,1] = fidArr
    dataTemp[:,5] = xs_rts[:,0]
    dataTemp[:,11] = xs_rts[:,1]
    dataTemp[:,12] = xs_rts[:,2]
    dataTemp[:,13] = lnArr

    return dataTemp, cov, covRts, likehood


def get_lane_id(dataVeh):
    initFrame = np.amin(dataVeh[:, 1])
    lastFrame = np.amax(dataVeh[:, 1])

    numFrame = int(lastFrame - initFrame + 1)

    frameArr = np.arange(initFrame, lastFrame+1)

    # lnInfo = dataVeh[0, 13] * np.ones((int(numFrame), ))

    # lnIdDiff = np.diff(dataVeh[:, 13])
    # lnChangeInd = np.where(lnIdDiff != 0)[0]
    # lnChangeInd = np.append(lnChangeInd, len(dataVeh[:, 13])-1)

    # if len(lnChangeInd) > 0:
    #     lastInd = int(0)
    #     for i, ind in enumerate(lnChangeInd):
    #         lnInfo[lastInd:ind] = dataVeh[ind, 13]
    #         lastInd = ind+1

    lnInfo = np.zeros(len(frameArr),)
    indLn = np.nonzero(dataVeh[:, 13])[0][0]

    if np.size(indLn) < 1:
        lastLane = 0
    else:
        lastLane = dataVeh[indLn, 13]

    for i, fm in enumerate(frameArr):
        ind = dataVeh[:,1] == fm
        if np.sum(ind)!=1:
            # print('warning')
            lnInfo[i] = lastLane
            # print(np.sum(ind))
        if np.sum(ind)==1:
            lnInfo[i] = dataVeh[ind, 13]
            lastLane = dataVeh[ind, 13]

    # print(lnInfo)

    return lnInfo

def motion_est(t, x, v, a):
    dt = 0.2
    numFrame = len(t)

    # position smoothing and v/a est
    initV = est_init_v(t, v, dt)
    cafilter = init_ca(dt, 0.02) # 0.005, 0.08
    cafilter.x = np.array([v[0], initV, 0])

    xs_k = []
    cov = np.zeros((numFrame, 3, 3))

    R_history = []
    diffTime = np.diff(t)
    diffTime = np.append(dt, diffTime)

    for i in range(numFrame):
        dt = diffTime[i]
        cafilter.Q = Q_discrete_white_noise(dim=3, dt=dt, var=2)
        cafilter.F = np.array([[1, dt, 0.5*dt*dt],
                              [0, 1,         dt],
                              [0, 0,          1]])
        cafilter.predict()
        # cafilter.R = measurement_noise_model(cafilter.x[0])
        
        cafilter.update(x[i])

        xs_k.append(cafilter.x)
        cov[i,:,:] = cafilter.P

        R_history.append(cafilter.R)

    xs_k = np.array(xs_k)
    xs_rts, _, _, _ = cafilter.rts_smoother(xs_k, cov)

    dataTemp = np.empty((numFrame, 5))
    dataTemp.fill(0)

    dataTemp[:,0] = t
    # dataTemp[:,1] = xs_rts[:,0]
    # dataTemp[:,2] = xs_rts[:,1]
    # dataTemp[:,3] = xs_rts[:,2]

    
    dataTemp[:,1] = xs_k[:,0]
    dataTemp[:,2] = xs_k[:,1]
    dataTemp[:,3] = xs_k[:,2]

    return dataTemp