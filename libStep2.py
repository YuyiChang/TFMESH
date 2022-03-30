import numpy as np
import time
from scipy.interpolate import interp1d
from scipy import interpolate

# Helper utility: find the start and ending points of continued logic true elements
def index_true_region(ind):
    indBreak = np.logical_xor(ind[1:], ind[:-1])
    indBreakRs = np.where(indBreak)[0]

    # if first sample satisfies target criterion (e.g.: first sample velocity is near-stop)
    if ind[0]:
        indBreakRs = np.append(-1, indBreakRs)
    # if last sample satisfies target criterion (e.g.: first sample velocity is near-stop)
    if ind[-1]:
        indBreakRs = np.append(indBreakRs, len(ind)-1)

    return indBreakRs[::2]+1, indBreakRs[::-2][::-1]

# Finding frame ID and position when vehicle is stopped
def index_stop_region(indNsStart, indNsEnd, indStop, vid):
    indStopStart, indStopEnd = np.zeros(len(indNsStart)), np.zeros(len(indNsStart))
    isStopped = True
    stopInfo = np.array([])


    if np.sum(indStop):
        indStopStartSeg, indStopEndSeg = index_true_region(indStop)

        for i in range(len(indNsStart)):
            isStartWithin = (indStopStartSeg >= indNsStart[i]) & (indStopStartSeg <= indNsEnd[i])
            isEndWithin = (indStopEndSeg >= indNsStart[i]) & (indStopEndSeg <= indNsEnd[i])

            isWithin = isStartWithin & isEndWithin

            if np.sum(isWithin) != 0:
                indStopStart[i] = indStopStartSeg[np.where(isWithin)[0][0]]
                indStopEnd[i] = indStopEndSeg[np.where(isWithin)[0][-1]]

        isValid = indStopStart>0
        indStopStart = indStopStart[isValid].astype(int)
        indStopEnd = indStopEnd[isValid].astype(int)
        indNsStart = indNsStart[isValid].astype(int)
        indNsEnd = indNsEnd[isValid].astype(int)

        stopInfo = np.vstack((np.ones((1,np.sum(isValid))) * vid, indNsStart, indStopStart, indStopEnd, indNsEnd))
        stopInfo = stopInfo.transpose().astype(int)

    else:
        isStopped = False
        indNsEnd, indNsStart = [], []

    return isStopped, stopInfo

# Connect moving and stopped parts with cubic spline
def spline_near_stop(stopInfo, dataVeh):
    def seg_spline_cubic2(x, y, xq):
        f = interp1d(x, y, kind='cubic')
        return f(xq)

    def seg_spline_cubic(x, y, xq):
        tck = interpolate.splrep(x, y, s=0)
        return interpolate.splev(xq, tck, der=0)


    data = dataVeh.copy()
    for i in range(np.shape(stopInfo)[0]):
        # set stop part
        data[stopInfo[i,2]:stopInfo[i,3]+1,5] = np.median(data[stopInfo[i,2]:stopInfo[i,3],5])
        data[stopInfo[i,2]:stopInfo[i,3]+1,[11,12]] = 0

        # before stop
        if stopInfo[i,1] != stopInfo[i,2]:
            ind = np.append(np.arange(0,stopInfo[i,1]+1), np.arange(stopInfo[i,2],len(data)))
            indQ = np.arange(stopInfo[i,1]-1,stopInfo[i,2]+1)
            data[indQ, 5] = seg_spline_cubic(data[ind, 1], data[ind, 5], data[indQ, 1])
            data[indQ, 11] = seg_spline_cubic(data[ind, 1], data[ind, 11], data[indQ, 1])
            data[indQ, 12] = seg_spline_cubic(data[ind, 1], data[ind, 12], data[indQ, 1])

        # after stop
        if stopInfo[i,3] != stopInfo[i,4]:
            ind = np.append(np.arange(stopInfo[i,2],stopInfo[i,3]+1), np.arange(stopInfo[i,4]+1,len(data)))
            indQ = np.arange(stopInfo[i,3]+1,stopInfo[i,4]+1)
            data[indQ, 5] = seg_spline_cubic(data[ind, 1], data[ind, 5], data[indQ, 1])
            data[indQ, 11] = seg_spline_cubic(data[ind, 1], data[ind, 11], data[indQ, 1])
            data[indQ, 12] = seg_spline_cubic(data[ind, 1], data[ind, 12], data[indQ, 1])

    return data

# Depreciated: use spline_near_stop()
def spline_near_stop_der(stopInfo, dataVeh):
    # def seg_spline_cubic(x, y, xq):
    #     tck = interpolate.splrep(x, y, s=0)
    #     return interpolate.splev(xq, tck, der=0)
    def seg_spline_fit(x, y):
        return interpolate.splrep(x, y, s=0)

    def seg_spline_eval(xq, tck, order):
        dt = 1/10
        if order == 0:
            eval = interpolate.splev(xq, tck, der=order)
        elif order == 1:
            eval = interpolate.splev(xq, tck, der=order)/dt
        else:
            eval = interpolate.splev(xq, tck, der=order)/dt/dt
        return eval

    data = dataVeh.copy()
    for i in range(np.shape(stopInfo)[0]):
        # set stop part
        data[stopInfo[i,2]:stopInfo[i,3]+1,5] = np.median(data[stopInfo[i,2]:stopInfo[i,3],5])
        data[stopInfo[i,2]:stopInfo[i,3]+1,[11,12]] = 0

        # before stop
        if stopInfo[i,1] != stopInfo[i,2]:
            ind = np.append(np.arange(0,stopInfo[i,1]+1), np.arange(stopInfo[i,2],len(data)))
            indQ = np.arange(stopInfo[i,1]-1,stopInfo[i,2]+1)
            tck = seg_spline_fit(data[ind, 1], data[ind, 5])
            data[indQ, 5] = seg_spline_eval(data[indQ, 1], tck, 0)
            data[indQ, 11] = seg_spline_eval(data[indQ, 1], tck, 1)
            data[indQ, 12] = seg_spline_eval(data[indQ, 1], tck, 2)

        # after stop
        if stopInfo[i,3] != stopInfo[i,4]:
            ind = np.append(np.arange(stopInfo[i,2],stopInfo[i,3]+1), np.arange(stopInfo[i,4]+1,len(data)))
            indQ = np.arange(stopInfo[i,3]+1,stopInfo[i,4]+1)
            tck = seg_spline_fit(data[ind, 1], data[ind, 5])
            data[indQ, 5] = seg_spline_eval(data[indQ, 1], tck, 0)
            data[indQ, 11] = seg_spline_eval(data[indQ, 1], tck, 1)
            data[indQ, 12] = seg_spline_eval(data[indQ, 1], tck, 2)

    return data

# Entry point for stop detection and handling
def ns_and_s_handle(dataVeh):
    # find simular position
    indSame = np.append(np.abs(dataVeh[1:, 5] - dataVeh[:-1, 5]) < 0.01, False)
    # find negative position chanegs
    indNeg = np.append(dataVeh[1:, 5] - dataVeh[:-1, 5] < 0, False)
    # find near-stop point
    # near-stop threshold = 4ft/s = 1.22 m/s
    indNs = dataVeh[:,11] <= 4
    # find stop point
    # stop threshold = 0.3ft/s = 0.09 m/s
    indS = dataVeh[:,11] <= 0.3
    ### to form stop detection logic
    indStop  = indSame | indNeg | indS

    indNsStart, indNsEnd = index_true_region(indNs)
    isStopped, stopInfo = index_stop_region(indNsStart, indNsEnd, indStop, dataVeh[0,0])

    dataVehS = dataVeh
    if isStopped:
        dataVehS = spline_near_stop(stopInfo, dataVeh)

    return dataVehS, isStopped, stopInfo
