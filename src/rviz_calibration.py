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
from scipy.signal import lfilter

# Get display
disp = Xlib.display.Display()

# Get directory paths
resources = pathlib.Path(__file__).parent.parent.joinpath('resources').resolve()
launch = pathlib.Path(__file__).parent.parent.joinpath('launch').resolve()
    
class InputOption:
    """
    Used for checkInput to use as options to display to the user
    """
    def __init__(self, optionName:str, optionDesc:str):
        """
        optionName: The str to match in the input\n
        optionDesc: A description of the action
        """
        self.name = optionName
        self.desc = optionDesc

    def __str__(self):
        return f'\t({self.name}): {self.desc}'

class ValueFilter:
    """
    Filters the values over time
    """
    def __init__(self, n:int, a:float, iterations:int):
        self.buff = np.zeros((2, iterations))
        self.n = n
        self.a = a
        self.b = [1.0 / n] * n 
        self.iterations = iterations
        self.index = 0

    def write(self, x, y):
        self.buff[0, self.index] = x
        self.buff[1, self.index] = y
        self.index = (self.index + 1) % self.iterations

    @property
    def value(self):
        """
        Filtered value
        """
        filx = lfilter(self.b, self.a, self.buff[0])
        fily = lfilter(self.b, self.a, self.buff[1])
        return filx.mean(), fily.mean()


def checkInput(prompt:object='',*options:InputOption, logType=ros.loginfo, acceptEmpty=None, emptyDesc='Continue'):
    """
    Prompts the user to enter an option. It also lists the options.\n
    Returns the text sent by the user
    """

    # Create the option to quit
    optQuit = InputOption('q', 'Exit the program')
    p = f'{prompt}'
    p += f'\n{optQuit}'

    # Add the additional options
    for opt in options:
        p += f'\n{opt}'

    # If there are no additional options, by default an empty message is valid. However if acceptEmpty is set to False, 
    # an empty message will fail and prompt the user again
    if (acceptEmpty is None and len(options) == 0) or acceptEmpty is True:
        optContinue = InputOption('Press [Enter]', emptyDesc)
        p += f'\n{optContinue}'

    # Promt the user until a valid option is chosen
    while True:
        logType(p)
        try:
            c = input()
        except EOFError: # ctrl+d will end the program
            raise KeyboardInterrupt()
        if c == 'q': # End the program
            raise KeyboardInterrupt()
        if (acceptEmpty is None and c == '' and len(options) == 0) or acceptEmpty is True: # Manage empty input
            break
        if c in [opt.name for opt in options]: # Check if valid option
            break
        ros.logerr(f'Invalid option selected: {(c if c != "" else "[Enter]")}') # Notify if invalid option
    
    return c

def checkTopic(topic:str, failMsg:str, successMsg:str):
    """
    Check if a specific namespace exists
    """
    while True:
        topics = ros.get_published_topics(topic)
        if len(topics) == 0:
            checkInput(f'{failMsg}\nPress enter to retry',logType=ros.logerr)
        else:
            ros.loginfo(successMsg)
            return

def askRVIZ():
    """
    Guide the user to setup rviz to work with the program
    """
    userInput = checkInput('Start RVIZ.', InputOption('s', 'Skip rviz procedure.'), acceptEmpty=True)
    if userInput == 's': # Allow skipping
        return

    userInput = checkInput('Please open the configuration found at .../cv_calibration/rviz/CompareDepthColor.rviz.',
        InputOption('s', 'Skip rviz procedure.'), acceptEmpty=True)
    if userInput == 's': # Allow skipping
        return

    ros.loginfo('Please make sure both camera viewers are activated and visible without overlap.')
    ros.loginfo('This program uses screen captures to calibrate the RVIZ streams. The bigger the camera viewers, the more precise the calibration will be.')
    ros.loginfo('This is also why it is important that the windows do not overlap and are not covered by other windows.')

    checkInput('Done?') # Wait for user input

def findFocusWithName(name:str):
    """
    For 3 seconds, check if the active window has the name 'name'
    """

    # Setup timeout
    end = ros.Time.now() + ros.Duration(secs=3)

    window = None

    while ros.Time.now() < end: # Wait until timeout
        window = disp.get_input_focus().focus # Get active window
        if window.get_wm_name() == 'Camera': # Check name
            return True, window

    ros.logwarn(f'Was expecting to find a window named "{name}" but found a window named "{window.get_wm_name()}".')
    return False, window
        
        

def findWindows():
    """
    Guide the user to find the color and depth windows
    """

    ros.loginfo('Once you are ready you will have 3 seconds after pressing enter to select the color camera viewer.')

    while True:
        checkInput('When ready, press enter and select the color camera window and wait for 3 seconds or until the window is found.')
        
        res, colorWindow = findFocusWithName('Camera') # Look for color window
        if res:
            userInput = checkInput('Found window!', InputOption('c', 'Continue'), InputOption('r', 'Retry'))
        else:
            userInput = checkInput('Continue anyway?', InputOption('c', 'Continue'), InputOption('r', 'Retry'),logType=ros.logwarn)
        if userInput == 'c':
            break
        ros.logwarn('Retrying...') # User chose retry
    
    ros.loginfo('Once you are ready you will have 3 seconds after pressing enter to select the depth camera viewer.')
    while True:
        checkInput('When ready, press enter and select the depth camera window and wait for 3 seconds or until the window is found.')

        res, depthWindow = findFocusWithName('Camera') # Look for depth window
        if res:
            userInput = checkInput('Found window!', InputOption('c', 'Continue'), InputOption('r', 'Retry'))
        else:
            userInput = checkInput('Continue anyway?', InputOption('c', 'Continue'), InputOption('r', 'Retry'),logType=ros.logwarn)
        if userInput == 'c':
            break
        ros.logwarn('Retrying...') # User chose retry
    
    return colorWindow, depthWindow

def isolateCamera(window):
    """
    Start from a capture of the entire camera window and isolate the camera with a 16:9 ratio
    """
    return getColorStandardizedRect(rvizRemoveBorders(boxFromWindow(window)))
    
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

def sendTransform(x:float, y:float, z:float, bc):
    """
    Send the depth transform
    """
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

    bc.sendTransform(tfstamped)

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
    Code 
    """
    ros.loginfo(f'Saving values to: {resources.joinpath("Results.txt").resolve()}')
    saveData()
    checkInput('Calibration paused.', InputOption('c', 'Resume calibration'))

if __name__ == '__main__':
    # Start node
    ros.init_node('rviz_calibration', anonymous=True)

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


