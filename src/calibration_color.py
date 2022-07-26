import rospy as ros
import cv2 as cv
import roslib
from tools.imageProcessor import adjustParam, processingMethod, imageProcessor, rectInt

def main():
    ros.init_node('calibration_color', anonymous=True)
    ros.loginfo('Starting color')
    colorProcessor = imageProcessor('/camera/color/image_raw', 'bgr8', processingMethod.THRESH_METHOD, thresh=adjustParam(190, ord('w'), ord('s'), 5))
    # colorProcessor = imageProcessor('/camera/color/image_rect_color', 'bgr8', processingMethod.THRESH_METHOD, thresh=adjustParam(190, ord('w'), ord('s'), 5))
    ros.loginfo('Started color')
    try:
        ros.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv.destroyAllWindows()

if __name__ == '__main__':
    main()
