import cv2
import depthai as dai
import numpy as np

class StereoInterface():
    def __init__(self) -> None:
        self.pipeline = dai.Pipeline()

        monoLeft = self.pipeline.create(dai.node.MonoCamera)
        monoRight = self.pipeline.create(dai.node.MonoCamera)

        xoutLeft = self.pipeline.create(dai.node.XLinkOut)
        xoutRight = self.pipeline.create(dai.node.XLinkOut)

        xoutLeft.setStreamName('left')
        xoutRight.setStreamName('right')

        monoLeft.setBoardSocket(dai.CameraBoardSocket.LEFT)
        monoLeft.setResolution(dai.MonoCameraProperties.SensorResolution.THE_720_P)
        monoRight.setBoardSocket(dai.CameraBoardSocket.RIGHT)
        monoRight.setResolution(dai.MonoCameraProperties.SensorResolution.THE_720_P)

        monoLeft.out.link(xoutLeft.input)
        monoRight.out.link(xoutRight.input)

        self.device = dai.Device(self.pipeline)

    
    def get_left(self):
        que = self.device.getOutputQueue(name='left', maxSize=4, blocking=False)
        data = que.tryGet()
        if data is None:
            return False
        return data.getCvFrame()


    def get_right_frame(self):
        que = self.device.getOutputQueue(name='right', maxSize=4, blocking=False)
        data = que.tryGet()
        if data is None:
            return False
        return data.getCvFrame()



    