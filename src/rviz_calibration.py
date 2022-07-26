from selectors import EpollSelector
from time import sleep

from mss import mss
import roslaunch
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


disp = Xlib.display.Display()
windowName = 'Help window'
resources = pathlib.Path(__file__).parent.parent.joinpath('resources').resolve()
    

def checkInput(prompt:object=''):
    c = input(prompt)
    if c == 'q':
        raise KeyboardInterrupt()
    return c

def checkTopic(topic:str, failMsg:str, successMsg:str):
    while True:
        topics = ros.get_published_topics(topic)
        if len(topics) == 0:
            ros.logerr(failMsg)
            checkInput('Press enter to retry')
        else:
            ros.loginfo(successMsg)
            return

def askRVIZ():
    ros.loginfo('Start RVIZ and press enter when it is open. You can skip this step by writing "s"')
    userInput = checkInput()
    if userInput == 's':
        return
    ros.loginfo('Please open the configuration found at .../cv_calibration/rviz/CompareDepthColor.rviz then press enter.')
    userInput = checkInput()
    if userInput == 's':
        return
    ros.loginfo('Please make sure both camera viewers are activated and visible without overlap.')
    ros.loginfo('This program uses screen captures to calibrate the RVIZ streams. The bigger the camera viewers, the more precise the calibration will be.')
    ros.loginfo('This is also why it is important that the windows do not overlap and are not covered by other windows.')
    ros.loginfo('Once this is done press enter (in the terminal you are running this program)')
    userInput = checkInput()
    if userInput == 's':
        return

def findWindows():
    ros.loginfo('Currently, this program only supports that the RVIZ camera viewers are all on the same screen.')
    ros.loginfo('If they are not on the same screen, please move them on a single screen.')
    ros.loginfo('Once you are ready you will have 3 seconds after pressing enter to select the color camera viewer.')

    while True:
        ros.loginfo('When ready, press enter and select the color camera window and wait for 3 seconds.')
        checkInput()
        sleep(3)
        colorWindow = disp.get_input_focus().focus
        ros.loginfo(f'Found {colorWindow.get_wm_name()}')
        ros.loginfo(f'With {boxFromWindow(colorWindow)}')
        ros.loginfo(f'No border {rvizRemoveBorders(boxFromWindow(colorWindow))}')
        ros.loginfo(f'Result {isolateCamera(colorWindow)}')
        ros.loginfo('Continue to next step (c), retry (r) or quit (q)?')
        userInput = checkInput()
        if userInput == 'c':
            break
        ros.loginfo('Retrying...')
    
    ros.loginfo('Once you are ready you will have 3 seconds after pressing enter to select the depth camera viewer.')
    while True:
        ros.loginfo('When ready, press enter and select the depth camera window and wait for 3 seconds.')
        checkInput()
        sleep(3)
        depthWindow = disp.get_input_focus().focus
        
        ros.loginfo(f'Found {depthWindow.get_wm_name()}')
        ros.loginfo(f'With {boxFromWindow(depthWindow)}')
        ros.loginfo(f'No border {rvizRemoveBorders(boxFromWindow(depthWindow))}')
        ros.loginfo(f'Result {isolateCamera(depthWindow)}')
        ros.loginfo('Continue to next step (c), retry (r) or quit (q)?')
        userInput = checkInput()
        if userInput == 'c':
            break
        ros.loginfo('Retrying...')
    
    return colorWindow, depthWindow

def isolateCamera(window):
    return getColorStandardizedRect(rvizRemoveBorders(boxFromWindow(window)))
    # return boxFromWindow(window)
    
if __name__ == '__main__':
    ros.init_node('rviz_calibration', anonymous=True)
    testAspectCorrection()


    try:
        ros.loginfo('At any time, write "q" and press enter to exit the program')
        # checkTopic('/camera', 'Please start the the ros_kortex_vision driver', 'Found the camera driver')
        askRVIZ()
        color, depth = findWindows()

        colorCap = screenCapture(isolateCamera(color))
        depthCap = screenCapture(isolateCamera(depth))
        colorProcessor = screenImageProcessor('color', 'bgr8', processingMethod.THRESH_METHOD, thresh=adjustParam(190, ord('w'), ord('s'), 5))
        depthProcessor = screenImageProcessor('depth', 'bgr8', processingMethod.THRESH_METHOD, thresh=adjustParam(190, ord('e'), ord('d'), 1))
        cv.namedWindow('color', cv.WINDOW_AUTOSIZE)
        sct = mss()
        box = {'top':0, 'left':0, 'width':100, 'height':100}
        input()
        while True:
            colorCap.box = isolateCamera(color)
            depthCap.box = isolateCamera(depth)
            colorCap.capture()
            depthCap.capture()
            cv.imshow('color', colorCap.getImg())
            colorProcessor.processImage(colorCap.getImg())
            depthProcessor.processImage(depthCap.getImg())
            if (cv.waitKey(1) & 0xFF) == ord('q'):
                cv.destroyAllWindows()
                break

    except KeyboardInterrupt:
        pass


