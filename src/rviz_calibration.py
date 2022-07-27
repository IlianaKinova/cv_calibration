from cProfile import run
from random import random
from typing import Dict, Tuple
import roslaunch
from mss import mss
import numpy as np
import cv2 as cv
import rospy as ros
from tools.Calibrator import Calibrator
from tools.utils import debugData, boxFromWindow, getColorStandardizedRect, rvizRemoveBorders, testAspectCorrection
from tools.screenCapture import screenCapture
import pathlib
import Xlib
import Xlib.display
from ewmh import EWMH
from PIL import Image
from tools.imageProcessor import screenImageProcessor, adjustParam, processingMethod, rectFloat, rectInt
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
import tf.transformations
from geometry_msgs.msg import TransformStamped

disp = Xlib.display.Display()
windowName = 'Help window'
resources = pathlib.Path(__file__).parent.parent.joinpath('resources').resolve()
launch = pathlib.Path(__file__).parent.parent.joinpath('launch').resolve()
    
class InputOption:
    def __init__(self, optionName:str, optionDesc:str):
        self.name = optionName
        self.desc = optionDesc

    def __str__(self):
        return f'\t({self.name}): {self.desc}'

def checkInput(prompt:object='',*options:InputOption, logType=ros.loginfo, acceptEmpty=None, emptyDesc='Continue'):
    optQuit = InputOption('q', 'Exit the program')
    p = f'{prompt}'
    p += f'\n{optQuit}'
    for opt in options:
        p += f'\n{opt}'
    if (acceptEmpty is None and len(options) == 0) or acceptEmpty is True:
        optContinue = InputOption('Press [Enter]', 'Continue')
        p += f'\n{optContinue}'
    while True:
        logType(p)
        try:
            c = input()
        except EOFError:
            raise KeyboardInterrupt()
        if c == 'q':
            raise KeyboardInterrupt()
        if (acceptEmpty is None and c == '' and len(options) == 0) or acceptEmpty is True:
            break
        if c in [opt.name for opt in options]:
            break
        ros.logerr(f'Invalid option selected: {(c if c != "" else "[Enter]")}')
    
    return c

def checkTopic(topic:str, failMsg:str, successMsg:str):
    while True:
        topics = ros.get_published_topics(topic)
        if len(topics) == 0:
            checkInput(f'{failMsg}\nPress enter to retry',logType=ros.logerr)
        else:
            ros.loginfo(successMsg)
            return

def askRVIZ():
    userInput = checkInput('Start RVIZ.', InputOption('s', 'Skip rviz procedure.'), acceptEmpty=True)
    if userInput == 's':
        return
    userInput = checkInput('Please open the configuration found at .../cv_calibration/rviz/CompareDepthColor.rviz.',
        InputOption('s', 'Skip rviz procedure.'), acceptEmpty=True)
    if userInput == 's':
        return
    ros.loginfo('Please make sure both camera viewers are activated and visible without overlap.')
    ros.loginfo('This program uses screen captures to calibrate the RVIZ streams. The bigger the camera viewers, the more precise the calibration will be.')
    ros.loginfo('This is also why it is important that the windows do not overlap and are not covered by other windows.')
    checkInput('Done?')

def findFocusWithName(name:str):
    end = ros.Time.now() + ros.Duration(secs=3)

    while ros.Time.now() < end:
        window = disp.get_input_focus().focus
        if window.get_wm_name() == 'Camera':
            return True, window
    ros.logwarn(f'Was expecting to find a window named "{name}" but found a window named "{window.get_wm_name()}".')
    return False, window
        
        

def findWindows():
    ros.loginfo('Once you are ready you will have 3 seconds after pressing enter to select the color camera viewer.')

    while True:
        checkInput('When ready, press enter and select the color camera window and wait for 3 seconds or until the window is found.')
        
        res, colorWindow = findFocusWithName('Camera')
        if res:
            userInput = checkInput('Found window!', InputOption('c', 'Continue'), InputOption('r', 'Retry'))
        else:
            userInput = checkInput('Continue anyway?', InputOption('c', 'Continue'), InputOption('r', 'Retry'),logType=ros.logwarn)
        if userInput == 'c':
            break
        ros.logwarn('Retrying...')
    
    ros.loginfo('Once you are ready you will have 3 seconds after pressing enter to select the depth camera viewer.')
    while True:
        checkInput('When ready, press enter and select the depth camera window and wait for 3 seconds or until the window is found.')

        res, depthWindow = findFocusWithName('Camera')
        if res:
            userInput = checkInput('Found window!', InputOption('c', 'Continue'), InputOption('r', 'Retry'))
        else:
            userInput = checkInput('Continue anyway?', InputOption('c', 'Continue'), InputOption('r', 'Retry'),logType=ros.logwarn)
        if userInput == 'c':
            break
        ros.logwarn('Retrying...')
    
    return colorWindow, depthWindow

def isolateCamera(window):
    return getColorStandardizedRect(rvizRemoveBorders(boxFromWindow(window)))
    # return boxFromWindow(window)
    
def getScore(x1:float, y1:float, w1:float, h1:float,
             x2:float, y2:float, w2:float, h2:float):
    avgOffset = ((x1+w1/2)-(x2+w2/2),(y1+h1/2)-(y2+h2/2))
    if w2 == 0 or h2 == 0:
        avgScale = 1
    else:
        avgScale = (w1/w2+h1/h2)/2
    return avgScale, avgOffset

xCalibrate = Calibrator(0.004, 0.0005)
yCalibrate = Calibrator(0.004, 0.0005)

def calibrate(xScore:float, yScore:float):
    return xCalibrate.compute(xScore) and yCalibrate.compute(yScore)

def runCalibration(colorRect:rectInt, depthRect:rectInt, colorSize:Tuple[int,int], depthSize:Tuple[int,int], calib:bool):
    colorRelRect = colorRect.toRelative(colorSize)
    depthRelRect = depthRect.toRelative(depthSize)
    scaleScore, (xScore, yScore) = getScore(colorRelRect.x, colorRelRect.y, colorRelRect.w, colorRelRect.h,
        depthRelRect.x, depthRelRect.y, depthRelRect.w, depthRelRect.h)
    
    xCalibrate.currentVal = xScore
    yCalibrate.currentVal = yScore
    if calib:
        res = calibrate(xScore, yScore)
    else:
        return False
    return res

def sendTransform(x:float, y:float, z:float, bc):
    # bc = StaticTransformBroadcaster()
    tfstamped = TransformStamped()
    tfstamped.header.stamp = ros.Time.now()
    tfstamped.header.frame_id = 'camera_link'
    tfstamped.child_frame_id = 'camera_depth_frame'
    tfstamped.transform.translation.x = x
    tfstamped.transform.translation.y = y
    tfstamped.transform.translation.z = z
    quat = tf.transformations.quaternion_from_euler(0,0,0)
    tfstamped.transform.rotation.x = quat[0]
    tfstamped.transform.rotation.y = quat[1]
    tfstamped.transform.rotation.z = quat[2]
    tfstamped.transform.rotation.w = quat[3]
    # colortfstamped = TransformStamped()
    # colortfstamped.header.stamp = ros.Time.now()
    # colortfstamped.header.frame_id = 'camera_link'
    # colortfstamped.child_frame_id = 'camera_color_frame'
    # colortfstamped.transform.translation.x = 0
    # colortfstamped.transform.translation.y = 0
    # colortfstamped.transform.translation.z = 0
    # colortfstamped.transform.rotation.x = quat[0]
    # colortfstamped.transform.rotation.y = quat[1]
    # colortfstamped.transform.rotation.z = quat[2]
    # colortfstamped.transform.rotation.w = quat[3]

    bc.sendTransform(tfstamped)
    # bc.sendTransform(colortfstamped)

def convertBoxToRectInt(box:Dict[str,float]):
    res = rectInt()
    res.x = box['left']
    res.y = box['top']
    res.w = box['width']
    res.h = box['height']
    return res

if __name__ == '__main__':
    ros.init_node('rviz_calibration', anonymous=True)
    testAspectCorrection()
    # tfbc = TransformBroadcaster()
    tfbc = StaticTransformBroadcaster()
    uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
    roslaunch.configure_logging(uuid)
    tracking_launch = roslaunch.parent.ROSLaunchParent(
        uuid, [str(launch.joinpath('color_tf.launch').resolve())])
    tracking_launch.start()
    

    sendTransform(random()*0.01,random()*0.01,random()*0.01, tfbc)

    try:
        # Guide the user
        ros.loginfo('At any time, write "q" and press enter to exit the program or press "ctrl+d" to force end of an input')

        # Check for the camera driver to be running
        checkTopic('/camera', 'Please start the the ros_kortex_vision driver', 'Found the camera driver')

        # Guide the user to setup rviz
        askRVIZ()

        # Make the user locate the proper windows
        color, depth = findWindows()

        # Initialiaze captures
        colorCap = screenCapture(isolateCamera(color))
        depthCap = screenCapture(isolateCamera(depth))

        # Initialize the processors
        colorProcessor = screenImageProcessor('color', 'bgr8', processingMethod.THRESH_METHOD, thresh=adjustParam(190, ord('w'), ord('s'), 5, (0,255)))
        depthProcessor = screenImageProcessor('depth', 'bgr8', processingMethod.THRESH_ERODE_METHOD,
            thresh=adjustParam(102, ord('e'), ord('d'), 1, (0,255)),
            erode=adjustParam(10, ord('r'), ord('f'), 1, (0, 100)))

        # Initialize the screen capture thingy
        sct = mss()

        # Make a debug window
        cv.namedWindow('Debug', cv.WINDOW_AUTOSIZE)
        

        # Main loop
        while True:
            dbg=np.full((600,400),255, np.uint8)
            debugData(dbg, 12, 0,
                xError=xCalibrate.error,
                xThreshold=xCalibrate.threshold,
                yError=yCalibrate.error,
                yThreshold=yCalibrate.threshold)
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
            keyPressed = cv.waitKey(20)
            if (keyPressed & 0xFF) == ord('q'):
                raise KeyboardInterrupt()

            doCalibrate = False
            if (keyPressed & 0xFF) == ord('c'):
                # Run the calibration algorithm
                doCalibrate = True

                # Adjust transform
                sendTransform(xCalibrate.output, yCalibrate.output, 0, tfbc)
                ros.loginfo_throttle(2000, 'Calibration complete!')
            isCalibrated = runCalibration(colorProcessor.rect, depthProcessor.rect, colorSize, depthSize, doCalibrate)

            colorProcessor.updateKeyPress(keyPressed)
            depthProcessor.updateKeyPress(keyPressed)


    except KeyboardInterrupt:
        cv.destroyAllWindows()
        ros.signal_shutdown('User stopped program')
        tracking_launch.shutdown()
        quit(0)


