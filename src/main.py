import depthai as dai
import cv2
import numpy as np

def getFrame(queue: dai.Device.getOutputQueue):
  # Get frame from queue
  frame = queue.get()
  # Convert frame to OpenCV format and return
  return frame.getCvFrame()

def getMonoCamera(pipeline: dai.Pipeline, isLeft: bool):
    # Configure mono camera
    mono = pipeline.createMonoCamera()
 
    # Set Camera Resolution
    mono.setResolution(dai.MonoCameraProperties.SensorResolution.THE_480_P)
 
    # Set FPS
    mono.setFps(15)

    # Map board socket to camera
    if isLeft:
        # Get left camera
        mono.setBoardSocket(dai.CameraBoardSocket.CAM_B)
    else :
        # Get right camera
        mono.setBoardSocket(dai.CameraBoardSocket.CAM_C)
    return mono

if __name__ == '__main__':
    pipeline = dai.Pipeline()
 
    # Set up left and right cameras
    monoLeft = getMonoCamera(pipeline, isLeft = True)
    monoRight = getMonoCamera(pipeline, isLeft = False)
 
    # Set output Xlink for left camera
    xoutLeft = pipeline.createXLinkOut()
    xoutLeft.setStreamName("left")
 
    # Set output Xlink for right camera
    xoutRight = pipeline.createXLinkOut()
    xoutRight.setStreamName("right")
  
    # Attach cameras to output Xlink
    monoLeft.out.link(xoutLeft.input)
    monoRight.out.link(xoutRight.input)

    with dai.Device(pipeline) as device:
        # Get output queues. 
        leftQueue = device.getOutputQueue(name="left", maxSize=1)
        rightQueue = device.getOutputQueue(name="right", maxSize=1)
    
        # Set display window name
        cv2.namedWindow("Stereo Pair")
        # Variable used to toggle between side by side view and one frame view. 
        sideBySide = True
        while True:
            # Get left frame
            leftFrame = getFrame(leftQueue)
            # Get right frame 
            rightFrame = getFrame(rightQueue)
        
            if sideBySide:
                # Show side by side view
                imOut = np.hstack((leftFrame, rightFrame))
            else : 
                # Show overlapping frames
                imOut = np.uint8(leftFrame/2 + rightFrame/2)
            # Display output image
            cv2.imshow("Stereo Pair", imOut)
            
            # Check for keyboard input
            key = cv2.waitKey(1)
            if key == ord('q'):
                # Quit when q is pressed
                break
            elif key == ord('t'):
                # Toggle display when t is pressed
                sideBySide = not sideBySide

    cv2.destroyAllWindows()
    