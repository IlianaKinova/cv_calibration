from __future__ import print_function
from random import Random
import rospy as ros
import cv2 as cv
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from tools.Calibrator import Calibrator
from tools.utils import debugData
from tools.imageProcessor import adjustParam, calibrationMethod, imageProcessor, rect
import roslib
roslib.load_manifest('cv_calibration')

def main():
    ros.init_node('calibrationDepth', anonymous=True)
    ros.loginfo('Starting depth')
    depthProcessor = imageProcessor('/camera/depth/image_raw', '16UC1', calibrationMethod.DEPTH_THRESH_METHOD,
    threshMin=adjustParam(0, ord('e'), ord('d'), 1),
    threshMax=adjustParam(255, ord('r'), ord('f'), 1),
    xMin=adjustParam(0,ord('x'),ord('z')),
    xMax=adjustParam(300,ord('v'),ord('c')),
    yMin=adjustParam(0,ord('t'),ord('g')),
    yMax=adjustParam(300,ord('y'),ord('h')),
    cannyMin=adjustParam(50,ord('u'),ord('j')),
    cannyMax=adjustParam(200,ord('i'),ord('k')))
    ros.loginfo('Started depth')

    try:
        ros.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv.destroyAllWindows()

if __name__ == '__main__':
    main()
