import rospy as ros
import cv2 as cv
from tools.imageProcessor import adjustParam, processingMethod, imageProcessor, rectInt
import roslib
# roslib.load_manifest('cv_calibration')

def main():
    ros.init_node('calibration_depth', anonymous=True)
    ros.loginfo('Starting depth')
    # depthProcessor = imageProcessor('/camera/depth/image_raw', '16UC1', processingMethod.DEPTH_THRESH_METHOD,
    # depthProcessor = imageProcessor('/camera/depth/image_rect_raw', '16UC1', processingMethod.DEPTH_THRESH_METHOD,
    depthProcessor = imageProcessor('/camera/depth_registered/sw_registered/image_rect', '16UC1', processingMethod.DEPTH_THRESH_METHOD,
    threshMin=adjustParam(200, ord('e'), ord('d'), 1),
    threshMax=adjustParam(255, ord('r'), ord('f'), 1),
    xMin=adjustParam(220,ord('x'),ord('z')),
    xMax=adjustParam(270,ord('v'),ord('c')),
    yMin=adjustParam(95,ord('t'),ord('g')),
    yMax=adjustParam(185,ord('y'),ord('h')),
    cannyMin=adjustParam(50,ord('u'),ord('j')),
    cannyMax=adjustParam(200,ord('i'),ord('k')),
    blurSize=adjustParam(1,ord('o'),ord('l'), 1))
    ros.loginfo('Started depth')

    try:
        ros.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv.destroyAllWindows()

if __name__ == '__main__':
    main()
