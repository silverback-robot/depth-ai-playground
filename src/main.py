import depthai as dai
import cv2
import numpy as np

pipeline = dai.Pipeline()

monoLeft = pipeline.createMonoCamera()
monoLeft.setBoardSocket(dai.CameraBoardSocket.LEFT)
monoRight = pipeline.createMonoCamera()
monoRight.setBoardSocket(dai.CameraBoardSocket.RIGHT)

xoutLeft = pipeline.createXLinkOut()
xoutLeft.setStreamName("left")

xoutRight = pipeline.createXLinkOut()
xoutRight.setStreamName("right")

monoLeft.out.link(xoutLeft.input)
monoRight.out.link(xoutRight.input)

with dai.Device(pipeline) as device:
    leftQueue = device.getOutputQueue(name="left", maxSize=1)
    leftFrame = leftQueue.get()

    rightQueue = device.getOutputQueue(name="right", maxSize=1)
    rightFrame = rightQueue.get()

    cv2.namedWindow("Stereo Pair")
    print(leftFrame.shape, rightFrame.shape)
    sideBySide = True

    while True:
        if sideBySide:
            imOut = np.hstack((leftFrame, rightFrame))
        else:
            imOut = np.uint8((leftFrame/2 + rightFrame/2))
        
        cv2.imshow("Stereo Pair", imOut)
        key = cv2.waitKey(1)
        if key == ord('q'):
            break
        elif key == ord('t'):
            sideBySide = not sideBySide
