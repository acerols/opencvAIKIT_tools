import cv2
import depthai as dai

import numpy as np
import glob

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

    square_size = 2.2
    pattern_size = (7, 7)

    pattern_points = np.zeros((np.prod(patten_size), 3), np.float32)
    pattern_points[:,:2] = np.indices(pattern_size).T.reshape(-1, 2)
    pattern_points *= square_size
    leftobjpoints = list()
    rightobjpoints = list()
    leftimgpoints = list()
    rightimgpoints = list()

    while True:
        leftFrame = get_left_CvFrame(device)
        rightFrame = get_right_CvFrame(device)

        if not leftFrame is None:
            cv2.imshow("left", leftFrame)
            ret, corner = cv2.findChessboardCorners(leftFrame, pattern_size)
            leftimgpoints.append(leftFrame)
        
        if not rightFrame is None:
            cv2.imshow("right", rightFrame)

        if cv2.waitKey(1) == ord('q'):
            break 

if __name__ == '__main__':
    main()