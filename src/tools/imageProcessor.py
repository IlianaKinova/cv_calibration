import enum
import functools
from numbers import Number
from operator import methodcaller
from typing import Any, Callable, Tuple, Union

from tomlkit import array
import rospy as ros
import cv2 as cv
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image

class rectFloat:
    def __init__(self):
        self.x = float(0)
        self.y = float(0)
        self.w = float(0)
        self.h = float(0)

    def toAbsolute(self, screenSize:'Tuple[int,int]'):
        res = rectInt()
        res.x = int(self.x * screenSize[0])
        res.w = int(self.w * screenSize[0])
        res.y = int(self.y * screenSize[1])
        res.h = int(self.h * screenSize[1])
        return res

class rectInt:
    def __init__(self, obj:Any=None):
        self.x = int(0)
        self.y = int(0)
        self.w = int(0)
        self.h = int(0)
        if obj is not None:
            self.x = int(obj.x)
            self.y = int(obj.y)
            self.w = int(obj.w)
            self.h = int(obj.h)

    def toRelative(self, screenSize:'Tuple[int,int]'):
        res = rectFloat()
        res.x = float(self.x) / screenSize[0]
        res.w = float(self.w) / screenSize[0]
        res.y = float(self.y) / screenSize[1]
        res.h = float(self.h) / screenSize[1]
        return res
        

import roslib
try:
    from tools.utils import debugData
    roslib.load_manifest('cv_calibration')
    from cv_calibration.msg import Rect
except Exception as e:
    # from utils import debugData
    class Rect(rectInt):
        pass
    ros.logerr(f'Failed to load ros modules:{e}')

class processingMethod(enum.Enum):
    THRESH_METHOD = 0
    DEPTH_THRESH_METHOD = 1
    DEPTH_BLOB_FINDER = 2

class adjustParam:
    def __init__(self, initVal: Number, increaseKey:int, decreaseKey:int, step:Number = 5):
        self.value = initVal
        self.increaseKey = increaseKey
        self.decreaseKey = decreaseKey
        self.step = step

    def update(self, key:int):
        if key & 0xFF == self.increaseKey:
            self.value+=self.step
        if key & 0xFF == self.decreaseKey:
            self.value-=self.step

    def __str__(self):
        return f'Up:`{chr(self.increaseKey)}` Down:`{chr(self.decreaseKey)}` Value:{self.value}'

class imageProcessor:
    def __init__(self, topic:str, encoding:str, method:processingMethod, **kwargs:adjustParam):
        """
        For method == calibrationMethod.THRESH_METHOD, args:\n
            -thresh\n
        For method == calibrationMethod.DEPTH_THRESH_METHOD, args:\n
            -threshMin\n
            -threshMax\n
            -xMin\n
            -xMax\n
            -yMin\n
            -yMax\n
            -cannyMin\n
            -cannyMax\n
            -blurSize\n
        """
        self.bridge = CvBridge()
        self.image_sub = ros.Subscriber(topic,Image,self.callback)
        self.name = topic.split('/')[2] # Get the stream name from the topic
        self.rect = Rect()
        self.method = method
        self.calibArgs = kwargs
        self.encoding = encoding
        self.isInit = False
        self.pub = ros.Publisher(f'/calib/{self.name}/rect', Rect)

    def init(self):
        if not self.isInit:
            cv.namedWindow(self.name, cv.WINDOW_AUTOSIZE)
            cv.namedWindow(f'{self.name}_canny', cv.WINDOW_AUTOSIZE)
            cv.namedWindow(f'{self.name}_filter', cv.WINDOW_AUTOSIZE)
            self.isInit = True

    def threshMethod(self, img:cv.Mat):
        threshVal = self.calibArgs['thresh'].value
        gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)

        ret, thresh = cv.threshold(gray,threshVal,255,cv.THRESH_BINARY_INV)
        cv.imshow(f'{self.name}_filter', thresh)

        # Canny Edge detection
        canny = cv.Canny(thresh, 50, 200)
        cv.imshow(f'{self.name}_canny', canny)

        # Find contours
        cnts = cv.findContours(canny, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)[-2:]
        cnts = cnts[0] if len(cnts) == 2 else cnts[1]
        x=0
        y=0
        w=0
        h=0
        for c in cnts:
            x, y, w, h = cv.boundingRect(c)

        # My coordinates
        try:
            cv.circle(img,(x,y),        10, (255,255,0), 5)
            cv.circle(img,(x+w,y),      10, (255,255,0), 5)
            cv.circle(img,(x+w,y+h),    10, (255,255,0), 5)
            cv.circle(img,(x,y+h),      10, (255,255,0), 5)
        except (UnboundLocalError):
            pass
        debugData(img, 20, (255,0,0), Threshold=str(threshVal),
            TopLeft=    (x,y),
            TopRight=   (x+w,y),
            BottomRight=(x+w,y+h),
            BottomLeft= (x,y+h))
        self.rect.x = x
        self.rect.y = y
        self.rect.w = w
        self.rect.h = h
        cv.imshow(self.name,img)

    def depthThreshMethod(self, img:cv.Mat):

        # Get the adjustable parameter values
        threshValMin =  self.calibArgs['threshMin'].value
        threshValMax =  self.calibArgs['threshMax'].value
        xMin =          self.calibArgs['xMin'].value
        xMax =          self.calibArgs['xMax'].value
        yMin =          self.calibArgs['yMin'].value
        yMax =          self.calibArgs['yMax'].value
        cannyMin =      self.calibArgs['cannyMin'].value
        cannyMax =      self.calibArgs['cannyMax'].value
        blurSize =      self.calibArgs['blurSize'].value

        # Turn the 16 bit grayscale to an 8 bit image, but also take the detected range of values (depths) and scale it up so it takes the entire 8 bits
        # Example, the min value is 0, the max value is 100, so we scale it up so that in the end, a value of 100 will be converted to 256
        delta = img.max() - img.min()
        img8 = (img - img.min() * (0xFFFFFFFF/delta) / 256).astype('uint8')

        blurred = cv.blur(img8,(blurSize,blurSize)) # I found that blurring the image a little helped get better edge detection

        ret, thresh = cv.threshold(blurred,threshValMin,threshValMax,cv.THRESH_BINARY_INV) # Threshold to isolate the depth we want to detect

        # Crop the selected area we want to detect
        cropped = np.zeros(thresh.shape, thresh.dtype)
        cropped[yMin:yMax, xMin:xMax] = thresh[yMin:yMax, xMin:xMax]


        # Canny Edge detection
        canny = cv.Canny(cropped, cannyMin, cannyMax)
        dbg_canny = np.zeros(canny.shape, canny.dtype)
        dbg_canny[:,:] = canny[:,:]
        # cv.rectangle(dbg_canny, (xMin,yMin), (xMax,yMax), (127), 2)
        cv.imshow(f'{self.name}_canny', dbg_canny)

        # Find contours
        cnts = cv.findContours(canny, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)[-2:]
        cnts = cnts[0] if len(cnts) == 2 else cnts[1]
        x=0
        y=0
        w=0
        h=0
        for c in cnts:
            x, y, w, h = cv.boundingRect(c)

        # Corodinates of the square
        try:
            cv.circle(thresh,(x,y),     10, (128), 5)
            cv.circle(thresh,(x+w,y),   10, (128), 5)
            cv.circle(thresh,(x+w,y+h), 10, (128), 5)
            cv.circle(thresh,(x,y+h),   10, (128), 5)
        except (UnboundLocalError):
            pass
        debugData(thresh, 10, (128),
            TopLeft=    (x,y),
            TopRight=   (x+w,y),
            BottomRight=(x+w,y+h),
            BottomLeft= (x,y+h),
            **self.calibArgs)
        cv.imshow(f'{self.name}_filter', thresh)
        cv.imshow(self.name, img8)
        self.rect.x = x
        self.rect.y = y
        self.rect.w = w
        self.rect.h = h

    def depthBlobFinder(self, img:cv.Mat):
        img8 = (img/256).astype('uint8')
        detector = cv.SimpleBlobDetector()
        points = detector.detect(img)
        cv.drawKeypoints(img8, points, np.array([]), (128), cv.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
        cv.imshow(self.name, img8)
    
    def processImage(self, input: cv.Mat):
        if self.method == processingMethod.THRESH_METHOD:
            self.threshMethod(input)
        if self.method == processingMethod.DEPTH_THRESH_METHOD:
            self.depthThreshMethod(input)
        if self.method == processingMethod.DEPTH_BLOB_FINDER:
            self.depthBlobFinder(input)


    def callback(self,data):
        self.init()
        key = cv.waitKey(3)
        if key == ord('q'):
            ros.signal_shutdown('User requested quit')
        for v in self.calibArgs.values():
            v.update(key)
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, self.encoding)
        except CvBridgeError as e:
            print(e)
        self.processImage(cv_image)
        self.pub.publish(self.rect)


class screenImageProcessor:
    def __init__(self,name:str, encoding:str, method:processingMethod, **kwargs:adjustParam):
        """
        For method == calibrationMethod.THRESH_METHOD, args:\n
            -thresh\n
        For method == calibrationMethod.DEPTH_THRESH_METHOD, args:\n
            -threshMin\n
            -threshMax\n
            -xMin\n
            -xMax\n
            -yMin\n
            -yMax\n
            -cannyMin\n
            -cannyMax\n
            -blurSize\n
        """
        self.name = name
        self.rect = Rect()
        self.method = method
        self.calibArgs = kwargs
        self.encoding = encoding
        self.isInit = False

    def init(self):
        if not self.isInit:
            cv.namedWindow(self.name, cv.WINDOW_AUTOSIZE)
            cv.namedWindow(f'{self.name}_canny', cv.WINDOW_AUTOSIZE)
            cv.namedWindow(f'{self.name}_filter', cv.WINDOW_AUTOSIZE)
            self.isInit = True

    def threshMethod(self, img:cv.Mat):
        threshVal = self.calibArgs['thresh'].value
        gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)

        ret, thresh = cv.threshold(gray,threshVal,255,cv.THRESH_BINARY_INV)
        cv.imshow(f'{self.name}_filter', thresh)

        # Canny Edge detection
        canny = cv.Canny(thresh, 50, 200)
        cv.imshow(f'{self.name}_canny', canny)

        # Find contours
        cnts = cv.findContours(canny, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)[-2:]
        cnts = cnts[0] if len(cnts) == 2 else cnts[1]
        x=0
        y=0
        w=0
        h=0
        for c in cnts:
            x, y, w, h = cv.boundingRect(c)

        # My coordinates
        try:
            cv.circle(img,(x,y),        10, (255,255,0), 5)
            cv.circle(img,(x+w,y),      10, (255,255,0), 5)
            cv.circle(img,(x+w,y+h),    10, (255,255,0), 5)
            cv.circle(img,(x,y+h),      10, (255,255,0), 5)
        except (UnboundLocalError):
            pass
        debugData(img, 20, (255,0,0), Threshold=str(threshVal),
            TopLeft=    (x,y),
            TopRight=   (x+w,y),
            BottomRight=(x+w,y+h),
            BottomLeft= (x,y+h))
        self.rect.x = x
        self.rect.y = y
        self.rect.w = w
        self.rect.h = h
        cv.imshow(self.name,img)

    def depthThreshMethod(self, img:cv.Mat):

        # Get the adjustable parameter values
        threshValMin =  self.calibArgs['threshMin'].value
        threshValMax =  self.calibArgs['threshMax'].value
        xMin =          self.calibArgs['xMin'].value
        xMax =          self.calibArgs['xMax'].value
        yMin =          self.calibArgs['yMin'].value
        yMax =          self.calibArgs['yMax'].value
        cannyMin =      self.calibArgs['cannyMin'].value
        cannyMax =      self.calibArgs['cannyMax'].value
        blurSize =      self.calibArgs['blurSize'].value

        # Turn the 16 bit grayscale to an 8 bit image, but also take the detected range of values (depths) and scale it up so it takes the entire 8 bits
        # Example, the min value is 0, the max value is 100, so we scale it up so that in the end, a value of 100 will be converted to 256
        delta = img.max() - img.min()
        img8 = (img - img.min() * (0xFFFFFFFF/delta) / 256).astype('uint8')

        blurred = cv.blur(img8,(blurSize,blurSize)) # I found that blurring the image a little helped get better edge detection

        ret, thresh = cv.threshold(blurred,threshValMin,threshValMax,cv.THRESH_BINARY_INV) # Threshold to isolate the depth we want to detect

        # Crop the selected area we want to detect
        cropped = np.zeros(thresh.shape, thresh.dtype)
        cropped[yMin:yMax, xMin:xMax] = thresh[yMin:yMax, xMin:xMax]


        # Canny Edge detection
        canny = cv.Canny(cropped, cannyMin, cannyMax)
        dbg_canny = np.zeros(canny.shape, canny.dtype)
        dbg_canny[:,:] = canny[:,:]
        # cv.rectangle(dbg_canny, (xMin,yMin), (xMax,yMax), (127), 2)
        cv.imshow(f'{self.name}_canny', dbg_canny)

        # Find contours
        cnts = cv.findContours(canny, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)[-2:]
        cnts = cnts[0] if len(cnts) == 2 else cnts[1]
        x=0
        y=0
        w=0
        h=0
        for c in cnts:
            x, y, w, h = cv.boundingRect(c)

        # Corodinates of the square
        try:
            cv.circle(thresh,(x,y),     10, (128), 5)
            cv.circle(thresh,(x+w,y),   10, (128), 5)
            cv.circle(thresh,(x+w,y+h), 10, (128), 5)
            cv.circle(thresh,(x,y+h),   10, (128), 5)
        except (UnboundLocalError):
            pass
        debugData(thresh, 10, (128),
            TopLeft=    (x,y),
            TopRight=   (x+w,y),
            BottomRight=(x+w,y+h),
            BottomLeft= (x,y+h),
            **self.calibArgs)
        cv.imshow(f'{self.name}_filter', thresh)
        cv.imshow(self.name, img8)
        self.rect.x = x
        self.rect.y = y
        self.rect.w = w
        self.rect.h = h

    def depthBlobFinder(self, img:cv.Mat):
        img8 = (img/256).astype('uint8')
        detector = cv.SimpleBlobDetector()
        points = detector.detect(img)
        cv.drawKeypoints(img8, points, np.array([]), (128), cv.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
        cv.imshow(self.name, img8)
    
    def processImage(self, input: cv.Mat):
        if self.method == processingMethod.THRESH_METHOD:
            self.threshMethod(input)
        if self.method == processingMethod.DEPTH_THRESH_METHOD:
            self.depthThreshMethod(input)
        if self.method == processingMethod.DEPTH_BLOB_FINDER:
            self.depthBlobFinder(input)
