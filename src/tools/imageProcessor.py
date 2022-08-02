import enum
from numbers import Number
import random
from typing import Any, Iterable, List, Tuple, Union

import cv2 as cv
import numpy as np
from tools.utils import debugData
from tools.ValueFilter import ValueFilter
import rospy as ros

class rectFloat:
    def __init__(self):
        self.x = float(0)
        self.y = float(0)
        self.w = float(0)
        self.h = float(0)

    def toAbsolute(self, screenSize:Tuple[int,int]):
        res = rectInt()
        res.x = int(self.x * screenSize[0])
        res.w = int(self.w * screenSize[0])
        res.y = int(self.y * screenSize[1])
        res.h = int(self.h * screenSize[1])
        return res

class rectInt:
    def __init__(self, obj:Union[Tuple[int,int,int,int],List[int],Any]=None, x:int=None, y:int=None, w:int=None, h:int=None):
        self.x = int(0)
        self.y = int(0)
        self.w = int(0)
        self.h = int(0)
        if obj is not None:
            if hasattr(obj, 'x') and hasattr(obj, 'y') and hasattr(obj, 'w') and hasattr(obj, 'h'):
                self.x = int(obj.x)
                self.y = int(obj.y)
                self.w = int(obj.w)
                self.h = int(obj.h)
                return
            if hasattr(obj, '__getitem__') and len(obj) >= 4:
                self.x = obj[0]
                self.y = obj[1]
                self.w = obj[2]
                self.h = obj[3]
                return
        if x is not None:
            self.x = x
        if y is not None:
            self.y = y
        if w is not None:
            self.w = w
        if h is not None:
            self.h = h

    def toRelative(self, screenSize:Tuple[int,int]):
        res = rectFloat()
        res.x = float(self.x) / screenSize[0]
        res.w = float(self.w) / screenSize[0]
        res.y = float(self.y) / screenSize[1]
        res.h = float(self.h) / screenSize[1]
        return res
        

class Rect(rectInt):
    pass

class processingMethod(enum.Enum):
    THRESH_METHOD = 0
    DEPTH_THRESH_METHOD = 1
    DEPTH_BLOB_FINDER = 2
    THRESH_IGNORE_BLACK_METHOD = 3
    THRESH_ERODE_METHOD = 4

class adjustParam:
    def __init__(self, initVal: Number, increaseKey:int, decreaseKey:int, step:Number = 5, range:Tuple[Number,Number]=None):
        self.value = initVal
        self.increaseKey = increaseKey
        self.decreaseKey = decreaseKey
        self.step = step
        self.range = range

    def update(self, key:int):
        if key & 0xFF == self.increaseKey:
            self.value+=self.step
        if key & 0xFF == self.decreaseKey:
            self.value-=self.step
        if self.range is not None:
            self.value = min(self.range[1], max(self.range[0], self.value))



    def __str__(self):
        return f'Up:`{chr(self.increaseKey)}` Down:`{chr(self.decreaseKey)}` Value:{self.value}'

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
        self.rect = rectInt()
        self.method = method
        self.calibArgs = kwargs
        self.encoding = encoding
        self.isInit = False
        self.filter = ValueFilter(2, 1, 10, 4)

    def init(self):
        if not self.isInit:
            cv.namedWindow(self.name, cv.WINDOW_AUTOSIZE)
            cv.namedWindow(f'{self.name}_canny', cv.WINDOW_AUTOSIZE)
            cv.namedWindow(f'{self.name}_filter', cv.WINDOW_AUTOSIZE)
            self.isInit = True

    def findBoundingBoxes(self, canny:cv.Mat):
        cnts = cv.findContours(canny, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)[-2:]
        cnts = cnts[0] if len(cnts) == 2 else cnts[1]
        for c in cnts:
            x, y, w, h = cv.boundingRect(c)
            yield rectInt(x=x, y=y, w=w, h=h)

    def findBiggestBox(self, boxes:Iterable[rectInt]):
        x, y, w, h = (0,0,0,0)
        area = 0
        for xx, yy, ww, hh in ((rect.x, rect.y, rect.w, rect.h) for rect in boxes):
            if ww * hh > area:
                if area != 0: # Don't yield a 0,0,0,0 box
                    # Yield the previously biggest box since it was never yielded
                    yield rectInt(x=x, y=y, w=w, h=h)

                x = xx
                y = yy
                w = ww
                h = hh
                area = w * h
                # To avoid yielding the biggest box multiple times, we don't yield here
            else: # Yield every box that isn't the biggest
                yield rectInt(x=xx, y=yy, w=ww, h=hh) # Yield to streamline the process only iterating through once
        yield rectInt(x=x, y=y, w=w, h=h) # Last is the result
        
            
    def filterMinMaxSize(self, boxes:Iterable[rectInt], minCoverage:float, maxCoverage:float, screenArea:int):
        for x, y, w, h in ((rect.x, rect.y, rect.w, rect.h) for rect in boxes):
            coverage = (w * h) / screenArea
            if minCoverage < coverage and coverage < maxCoverage:
                yield rectInt(x=x, y=y, w=w, h=h) # Yield to streamline the process only iterating through once

    def findFillRatio(self, box:rectInt, thresh:cv.Mat):
        return 1.0 - (np.average(thresh[box.y:box.y+box.h, box.x:box.x+box.w]) / 255)

    def filterThreshFillRatio(self, boxes:Iterable[rectInt], thresh:cv.Mat, minFilling:float=0.9):
        for x, y, w, h in ((rect.x, rect.y, rect.w, rect.h) for rect in boxes):
            if self.findFillRatio(rectInt((x,y,w,h)), thresh) > minFilling:
                yield rectInt(x=x, y=y, w=w, h=h)


    def threshMethod(self, img:cv.Mat, ignoreBlack:bool = False):
        threshVal = self.calibArgs['thresh'].value
        gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)

        ret, thresh = cv.threshold(gray,threshVal,255,cv.THRESH_BINARY_INV)
        if ignoreBlack:
            for x, column in enumerate(gray):
                for y, p in enumerate(column):
                    if p == 0:
                        thresh[x, y] = 0
        cv.imshow(f'{self.name}_filter', thresh)

        # Canny Edge detection
        canny = cv.Canny(thresh, 50, 200)
        cv.imshow(f'{self.name}_canny', canny)

        # Find contours
        boxes = self.findBoundingBoxes(canny)
        sized = self.filterMinMaxSize(boxes, 0.05, 0.5, img.shape[0]*img.shape[1])
        filled = self.filterThreshFillRatio(sized, thresh)
        biggest = self.findBiggestBox(filled) # This still returns every box, but the last element is the biggest box
        # This is so that the entire process stays in generators so that iteration happens only once

        # Draw all boxes
        for x, y, w, h in ((rect.x, rect.y, rect.w, rect.h) for rect in biggest):
            color = (random.randrange(0, 255), random.randrange(0, 255), 127)
            cv.rectangle(img, (x,y), (x+w, y+h), color, 1)
            cv.circle(img,(x+w,y),      5, color, 1)
            cv.circle(img,(x,y),        5, color, 1)
            cv.circle(img,(x+w,y+h),    5, color, 1)
            cv.circle(img,(x,y+h),      5, color, 1)

        # Filter values
        ros.loginfo_throttle(1, f'Before {x}')
        self.filter.write(x, y, w, h)
        x, y, w, h = tuple([int(round(v)) for v in self.filter.value])
        ros.loginfo_throttle(1, f'After {x}')
        fillPercent = self.findFillRatio(rectInt((x,y,w,h)), thresh)

        # Last box is biggest
        cv.rectangle(img, (x,y), (x+w, y+h), (255,255,0), 2)
        cv.circle(img,(x,y),        10, (255,255,0), 5)
        cv.circle(img,(x+w,y),      10, (255,255,0), 5)
        cv.circle(img,(x+w,y+h),    10, (255,255,0), 5)
        cv.circle(img,(x,y+h),      10, (255,255,0), 5)


        debugData(img, 20, (255,0,0),
            TopLeft=    (x,y),
            TopRight=   (x+w,y),
            BottomRight=(x+w,y+h),
            BottomLeft= (x,y+h),
            FillPercent=fillPercent,
            **self.calibArgs)
        self.rect.x = x
        self.rect.y = y
        self.rect.w = w
        self.rect.h = h
        cv.imshow(self.name,img)

    def threshErodeMethod(self, img:cv.Mat):
        threshVal = self.calibArgs['thresh'].value
        erodeVal = self.calibArgs['erode'].value
        gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)

        ret, thresh = cv.threshold(gray,threshVal,255,cv.THRESH_BINARY_INV)
        cv.imshow(f'{self.name}_filter', thresh)

        kernel = np.ones((erodeVal,erodeVal),np.uint8)
        eroded = cv.erode(thresh, kernel)

        # Canny Edge detection
        canny = cv.Canny(eroded, 50, 200)
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
            BottomLeft= (x,y+h),
            **self.calibArgs)
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

        _, thresh = cv.threshold(blurred,threshValMin,threshValMax,cv.THRESH_BINARY_INV) # Threshold to isolate the depth we want to detect

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
    
    def updateKeyPress(self, key:int):
        for k, arg in self.calibArgs.items():
            arg.update(key)

    def destroyWindows(self):
        cv.destroyWindow(self.name)
        cv.destroyWindow(f'{self.name}_filter')
        cv.destroyWindow(f'{self.name}_canny')
        self.isInit = False

    def processImage(self, input: cv.Mat):
        self.init()
        if self.method == processingMethod.THRESH_METHOD:
            self.threshMethod(input)
        if self.method == processingMethod.DEPTH_THRESH_METHOD:
            self.depthThreshMethod(input)
        if self.method == processingMethod.DEPTH_BLOB_FINDER:
            self.depthBlobFinder(input)
        if self.method == processingMethod.THRESH_IGNORE_BLACK_METHOD:
            self.threshMethod(input, True)
        if self.method == processingMethod.THRESH_ERODE_METHOD:
            self.threshErodeMethod(input)
