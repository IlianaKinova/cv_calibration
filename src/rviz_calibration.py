# Ros imports
import rospy as ros
import roslaunch
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster

# Image related imports
import cv2 as cv
from mss import mss
import Xlib
import Xlib.display

# Tools imports
from tools.imageProcessor import screenImageProcessor, adjustParam, processingMethod, rectInt
from tools.Calibrator import Calibrator
from tools.utils import debugData, checkInput, InputOption, sendTransform
from tools.screenCapture import screenCapture
from tools.UserHelp import askRVIZ, checkTopic, findWindows
from tools.ValueFilter import ValueFilter
from tools.WindowOperations import isolateCamera

# Other imports
from random import random
from typing import Dict, Tuple
import numpy as np
import pathlib

# Get display
disp = Xlib.display.Display()

# Get directory paths
resources = pathlib.Path(__file__).parent.parent.joinpath('resources').resolve()
launch = pathlib.Path(__file__).parent.parent.joinpath('launch').resolve()
    
def getScore(x1:float, y1:float, w1:float, h1:float,
             x2:float, y2:float, w2:float, h2:float):
    """
    Get the scale and position offsets of the rectangles
    """
    # Average the offsets
    avgOffset = ((x1+w1/2)-(x2+w2/2),(y1+h1/2)-(y2+h2/2))
    if w2 == 0 or h2 == 0: # Avoid divided by 0 error
        avgScale = 1
    else: 
        # Average the scale
        avgScale = (w1/w2+h1/h2)/2
    return avgScale, avgOffset

# Setup the calibrators
xCalibrate = Calibrator(0.004, 0.0005)
yCalibrate = Calibrator(0.004, 0.0005)

# Value filter for the depth stream
vFilter = ValueFilter(5, 1, 10)

def calibrate(xScore:float, yScore:float):
    """
    Compute the calibrators
    """
    resx = xCalibrate.compute(xScore)
    resy = yCalibrate.compute(yScore)
    return resx and resy

def runCalibration(colorRect:rectInt, depthRect:rectInt, colorSize:Tuple[int,int], depthSize:Tuple[int,int], calib:bool):
    """
    Run the calibration algorithm
    """
    # Transpose the rectangles from absolute (pixel) to relative (0.0-1.0) values
    colorRelRect = colorRect.toRelative(colorSize)
    depthRelRect = depthRect.toRelative(depthSize)

    # Calculate the scores
    scaleScore, (xScore, yScore) = getScore(colorRelRect.x, colorRelRect.y, colorRelRect.w, colorRelRect.h,
        depthRelRect.x, depthRelRect.y, depthRelRect.w, depthRelRect.h)
    
    # Filter scores
    vFilter.write(xScore, yScore)
    xScore, yScore = vFilter.value

    # Update calibration values
    xCalibrate.currentVal = xScore
    yCalibrate.currentVal = yScore

    if calib: # Calibrate
        res = calibrate(xScore, yScore)
    else:
        return False
    return res



def convertBoxToRectInt(box:Dict[str,float]):
    """
    Simple conversion from a window geometry to a rectInt
    """
    res = rectInt()
    res.x = box['left']
    res.y = box['top']
    res.w = box['width']
    res.h = box['height']
    return res

def saveData():
    """
    Save the programed values
    """
    file = resources.joinpath('Results.txt').resolve()
    with open(file, 'w') as writer:
        writer.write('OUTPUT VALUES\n')
        writer.write(f'x y z: {xCalibrate.output} {yCalibrate.output} 0\n\n')
        writer.write('PARAMETERS\n')
        writer.write('\nCOLOR\n')
        for key, val in colorProcessor.calibArgs.items():
            writer.write(f'{key}: {val.value}\n')
        writer.write('\nDEPTH\n')
        for key, val in depthProcessor.calibArgs.items():
            writer.write(f'{key}: {val.value}\n')
        writer.close()
        
def afterCalibration():
    """
    Code to run after calibration is stopped
    """
    ros.loginfo(f'Saving values to: {resources.joinpath("Results.txt").resolve()}')
    saveData()
    checkInput('Calibration paused.', InputOption('c', 'Resume calibration'))

if __name__ == '__main__':
    # Start node
    ros.init_node('rviz_calibration', anonymous=True)

    

    try:
        # Guide the user
        ros.loginfo('At any time, write "q" and press enter to exit the program or press "ctrl+d" to force end of an input')

        # Check for the camera driver to be running
        checkTopic('/camera', 'Please start the the ros_kortex_vision driver', 'Found the camera driver')

        # Init depth transform publisher
        tfbc = StaticTransformBroadcaster()

        # Start color transform in parallel
        uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        roslaunch.configure_logging(uuid)
        tracking_launch = roslaunch.parent.ROSLaunchParent(
            uuid, [str(launch.joinpath('color_tf.launch').resolve())])
        tracking_launch.start()

        # Send an initial transform to allow rviz to enable the point cloud
        sendTransform(random()*0.01,random()*0.01,random()*0.01, tfbc)

        # Guide the user to setup rviz
        askRVIZ()

        # Make the user locate the proper windows
        color, depth = findWindows(disp)

        # Initialiaze captures
        colorCap = screenCapture(isolateCamera(color))
        depthCap = screenCapture(isolateCamera(depth))

        # Initialize the processors
        colorProcessor = screenImageProcessor('color', 'bgr8', processingMethod.THRESH_METHOD, thresh=adjustParam(100, ord('w'), ord('s'), 5, (0,255)))
        depthProcessor = screenImageProcessor('depth', 'bgr8', processingMethod.DEPTH_THRESH_METHOD,
            thresh=adjustParam(102, ord('e'), ord('d'), 1, (0,255)))

        # Initialize the screen capture thingy
        sct = mss()

        # Make a debug window
        cv.namedWindow('Debug', cv.WINDOW_AUTOSIZE)

        # Main loop
        while True:
            # Debug data
            dbg=np.full((600,400),255, np.uint8)
            debugData(dbg, 12, 0,
                xError=xCalibrate.error,
                xThreshold=xCalibrate.threshold,
                yError=yCalibrate.error,
                yThreshold=yCalibrate.threshold,
                ExitProgram='Press q',
                Calibrate='Press and hold c',
                ResetCalibration='Press x',
                StopCalibration='Press z')
            cv.imshow('Debug', dbg)

            # Update the window positions
            colorCap.box = isolateCamera(color)
            depthCap.box = isolateCamera(depth)

            # Capture the windows
            colorCap.capture()
            depthCap.capture()

            # Process the captures
            colorProcessor.processImage(colorCap.getImg())
            depthProcessor.processImage(depthCap.getImg())

            # Convert the sizes for relative scaling
            colorSize = (colorCap.box['width'], colorCap.box['height'])
            depthSize = (depthCap.box['width'], depthCap.box['height'])

            # Provide a way to exit code at any time (while the windows are active)
            keyPressed = cv.waitKey(3)
            if (keyPressed & 0xFF) == ord('q'):
                raise KeyboardInterrupt()

            doCalibrate = False
            if (keyPressed & 0xFF) == ord('c'):
                # Run the calibration algorithm
                doCalibrate = vFilter.index == 0

                # Adjust transform
                sendTransform(xCalibrate.output, yCalibrate.output, 0, tfbc)
                # ros.loginfo_throttle(2000, 'Calibration complete!')
            isCalibrated = runCalibration(colorProcessor.rect, depthProcessor.rect, colorSize, depthSize, doCalibrate)

            if (keyPressed & 0xFF) == ord('x'):
                xCalibrate.reset()
                yCalibrate.reset()
                sendTransform(0,0,0, tfbc)
                ros.loginfo('Reset calibration parameters')

            if (keyPressed & 0xFF) == ord('z'):
                colorProcessor.destroyWindows()
                depthProcessor.destroyWindows()
                afterCalibration()


            # Update the adjustable parameters with the last pressed key
            colorProcessor.updateKeyPress(keyPressed)
            depthProcessor.updateKeyPress(keyPressed)


    except KeyboardInterrupt: # End program
        cv.destroyAllWindows()
        ros.signal_shutdown('User stopped program')
        tracking_launch.shutdown() # Shutdown the color transform
        quit(0)


