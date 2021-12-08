import cv2
import depthai as dai

import numpy as np
import glob

from numpy.lib.arraysetops import isin

def create_pipeline():
    pipeline = dai.Pipeline()

    monoLeft = pipeline.create(dai.node.MonoCamera)
    monoRight = pipeline.create(dai.node.MonoCamera)

    xoutLeft = pipeline.create(dai.node.XLinkOut)
    xoutRight = pipeline.create(dai.node.XLinkOut)

    xoutLeft.setStreamName('left')
    xoutRight.setStreamName('right')

    monoLeft.setBoardSocket(dai.CameraBoardSocket.LEFT)
    monoLeft.setResolution(dai.MonoCameraProperties.SensorResolution.THE_720_P)
    monoRight.setBoardSocket(dai.CameraBoardSocket.RIGHT)
    monoRight.setResolution(dai.MonoCameraProperties.SensorResolution.THE_720_P)

    monoLeft.out.link(xoutLeft.input)
    monoRight.out.link(xoutRight.input)

    return pipeline

def get_left_CvFrame(device):
    qLeft = device.getOutputQueue(name='left', maxSize=4, blocking=False)
    inLeft = qLeft.tryGet()
    if inLeft is None:
        return None
    return inLeft.getCvFrame()

def get_right_CvFrame(device):
    qRight = device.getOutputQueue(name='right', maxSize=4, blocking=False)
    inRight = qRight.tryGet()
    if inRight is None:
        return None

    return inRight.getCvFrame()

def main():
    pipeline = create_pipeline()
    device = dai.Device(pipeline)

    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.01)
    objp = np.zeros((6*7, 3), )

    square_size = 2.4
    pattern_size = (7, 10)

    pattern_points = np.zeros((np.prod(pattern_size), 3), np.float32)
    pattern_points[:,:2] = np.indices(pattern_size).T.reshape(-1, 2)
    pattern_points *= square_size
    leftobjpoints = list()
    rightobjpoints = list()
    leftimgpoints = list()
    rightimgpoints = list()

    isInfo = True

    while True:
        leftFrame = get_left_CvFrame(device)
        rightFrame = get_right_CvFrame(device)

        if not leftFrame is None:
            if isInfo:
                isInfo = False
                print(f"camera shape ({leftFrame.shape})")

            #cv2.imshow("left", leftFrame)
            corner = None
            ret, corner = cv2.findChessboardCorners(leftFrame, pattern_size)
            if not corner is None:
                print(corner)
                cv2.cornerSubPix(leftFrame, corner, (5, 5), (-1, -1), criteria)
                leftimgpoints.append(corner.reshape(-1, 2))
                leftobjpoints.append(pattern_points)

        
        if not rightFrame is None:
            #cv2.imshow("right", rightFrame)
            corner = None
            ret, corner = cv2.findChessboardCorners(rightFrame, pattern_size)
            if not corner is None:
                cv2.cornerSubPix(rightFrame, corner, (5, 5), (-1, -1), criteria)
                rightimgpoints.append(corner.reshape(-1, 2))
                rightobjpoints.append(pattern_points)

        if not rightFrame is None and not leftFrame is None:
            con = np.concatenate([leftFrame, rightFrame], 1)
            #con = np.zeros((leftFrame.shape[0], leftFrame.shape[1], 3), dtype=np.uint8)
            #con[:, :, 2] = leftFrame
            #con[:, :, 1] = rightFrame

            
            cv2.imshow("concatenate", con)

        if cv2.waitKey(500) == ord('q'):
            break 
    
        
    if len(leftimgpoints) < len(leftobjpoints):
        leftobjpoints = leftobjpoints[:len(leftimgpoints)]
    elif len(leftimgpoints) > len(leftobjpoints):
        leftimgpoints = leftimgpoints[:len(leftobjpoints)]

    if not len(rightimgpoints) is len(rightobjpoints):
        rightobjpoints = rightobjpoints[:len(rightobjpoints)]

    leftFrame = get_left_CvFrame(device)
    rightFrame = get_right_CvFrame(device)


    ret, left_mtx, left_dist, left_rvec, left_tvecs = cv2.calibrateCamera(leftobjpoints, leftimgpoints, leftFrame.shape[::-1], None, None)
    ret, right_mtx, right_dist, right_rvec, right_tvecs = cv2.calibrateCamera(
                            rightobjpoints, rightimgpoints, rightFrame.shape[::-1], None, None
    )

    np.save("left_mtx", left_mtx)
    np.save("left_dist", left_dist)
    np.save("right_mtx", right_mtx)
    np.save("right_dist", right_dist)

    print("RMS = ", ret)
    print("mtx = \n", left_mtx)
    print("dist = ", left_dist.ravel())

    print("RMS = ", ret)
    print("mtx = \n", right_mtx)
    print("dist = ", right_dist.ravel())

if __name__ == '__main__':
    main()