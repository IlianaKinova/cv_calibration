from __future__ import print_function
import rospy as ros
import cv2 as cv
from tools.imageProcessor import adjustParam, calibrationMethod, imageProcessor, rect
import roslib
roslib.load_manifest('cv_calibration')

def main():
    ros.init_node('calibrationColor', anonymous=True)
    ros.loginfo('Starting color')
    colorProcessor = imageProcessor('/camera/color/image_raw', 'bgr8', calibrationMethod.THRESH_METHOD, thresh=adjustParam(190, ord('w'), ord('s'), 5))
    ros.loginfo('Started color')
    try:
        ros.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv.destroyAllWindows()

if __name__ == '__main__':
    main()
