import rospy as ros
import cv2 as cv
import numpy as np
from tools.Calibrator import Calibrator
from tools.imageProcessor import rectInt, rectFloat
from tools.RobotController import RobotController
import message_filters
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import TransformStamped, Transform, Vector3, Quaternion

import roslaunch


import roslib
try:
    roslib.load_manifest('cv_calibration')
    from cv_calibration.msg import Rect
    from kortex_driver.msg import *
    from kortex_driver.srv import *
except Exception as e:
    class Rect(rectInt):
        pass
    print(f'Failed to load ros modules:{e}')

ctl = RobotController()
tf2Pub = ros.Publisher('/tf_static', TFMessage, queue_size=3)

def process(y:float):
    return y * -1.245 - 5 + 3 ** (y + 3)

def mytest():
    calib = Calibrator(0.2, 0.0001)
    proc = process(0)
    while True:
        proc = process(calib.output)
        res = calib.compute(proc)
        ros.loginfo(f'Process: {proc}\tOutput: {calib.output}\tError: {calib.error}\tTuning: {calib.crawlingSpeed}')
        if res:
            break
    ros.logdebug("Test success!")

    
def getScore(x1:float, y1:float, w1:float, h1:float,
             x2:float, y2:float, w2:float, h2:float):
    avgOffset = ((x1+w1/2)-(x2+w2/2),(y1+h1/2)-(y2+h2/2))
    if w2 == 0 or h2 == 0:
        avgScale = 1
    else:
        avgScale = (w1/w2+h1/h2)/2
    return avgScale, avgOffset

xCalibrate = Calibrator(0.4, 0.01)
yCalibrate = Calibrator(0.4, 0.01)

def calibrate(xScore:float, yScore:float):
    return xCalibrate.compute(xScore) and yCalibrate.compute(yScore)

def calibrateCB(color:Rect, depth:Rect):
    colorRect = rectInt(color).toRelative((ctl.colorInfo.width, ctl.colorInfo.height))
    depthRect = rectInt(depth).toRelative((ctl.depthInfo.width, ctl.depthInfo.height))
    scaleScore, (xScore, yScore) = getScore(colorRect.x, colorRect.y, colorRect.w, colorRect.h,
        depthRect.x, depthRect.y, depthRect.w, depthRect.h)
    res = calibrate(xScore, yScore)
    # ctl.setTranslation((xCalibrate.output, yCalibrate.output))
    msg = TFMessage()

    tf = TransformStamped()
    tf.transform.translation.x = xCalibrate.output
    tf.transform.translation.y = yCalibrate.output
    tf.child_frame_id = 'camera_depth_frame'
    tf.header.frame_id = 'camera_link'
    msg.transforms.append(tf)
    tf2Pub.publish(msg)
    ros.loginfo(f'x:{xScore}\ty:{yScore}\ts:{scaleScore}')
    ros.loginfo(f'out x:{xCalibrate.output}\tout y:{yCalibrate.output}')
    if res:
        ros.loginfo('Calibration done!')
        return


def main():
    print('hi')
    ros.init_node('calibration')
    ros.logdebug('Starting calibrator')
    ctl.init()
    ctl.getExtrinsic()
    ctl.updateCameraInfo()
    # ctl.exploreVisionConfig()

    colorSub = message_filters.Subscriber('/calib/color/rect', Rect)
    depthSub = message_filters.Subscriber('/calib/depth/rect', Rect)

    ts = message_filters.TimeSynchronizer([colorSub, depthSub], 1)
    ts.registerCallback(calibrateCB)


    try:
        ros.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv.destroyAllWindows()

if __name__ == '__main__':
    main()
