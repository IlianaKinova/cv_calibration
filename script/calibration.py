from __future__ import print_function
import enum
from numbers import Number
from random import Random
import rospy as ros
import cv2 as cv
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from tools.utils import debugData
import roslib
roslib.load_manifest('cv_calibration')

class rect:
    def __init__(self, x:float, y:float, w:float, h:float):
        self.x = x
        self.y = y
        self.w = w
        self.h = h

class calibrationMethod(enum.Enum):
    THRESH_METHOD = 1
    DEPTH_THRESH_METHOD = 2

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
        return f'Up:`{self.increaseKey}` Down:`{self.decreaseKey}` Value:{self.value}'

class imageProcessor:

    def __init__(self, topic:str, encoding:str, method:calibrationMethod, **kwargs:adjustParam):
        """For method == calibrationMethod.THRESH_METHOD, args:
            -thresh"""
        self.bridge = CvBridge()
        self.image_sub = ros.Subscriber(topic,Image,self.callback)
        self.name = topic.split('/')[2] # Get the stream name from the topic
        self.rect = rect(0,0,0,0)
        self.method = method
        self.calibArgs = kwargs
        self.encoding = encoding

    def threshMethod(self, img:cv.Mat):
        threshVal = self.calibArgs['thresh'].value
        gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)

        ret, thresh = cv.threshold(gray,threshVal,255,cv.THRESH_BINARY_INV)
        # cv.imshow(f'{self.name}_filter', thresh)

        # Canny Edge detection
        canny = cv.Canny(thresh, 50, 200)
        # cv.imshow(f'{self.name}_canny', canny)

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
            cv.circle(img,(x,y), 10, (255,255,0), 5)
            cv.circle(img,(x+w,y), 10, (255,255,0), 5)
            cv.circle(img,(x+w,y+h), 10, (255,255,0), 5)
            cv.circle(img,(x,y+h), 10, (255,255,0), 5)
        except (UnboundLocalError):
            pass
        debugData(img, 20, (255,0,0), Threshold=str(threshVal),
            TopLeft=    (x,y),
            TopRight=   (x+w,y),
            BottomRight=(x+w,y+h),
            BottomLeft= (x,y+h))
        self.rect = rect(x,y,w,h)
        cv.imshow(self.name,img)

    def depthThreshMethod(self, img:cv.Mat):
        threshValMin = self.calibArgs['threshMin'].value
        threshValMax = self.calibArgs['threshMax'].value
        xMin = self.calibArgs['xMin'].value
        xMax = self.calibArgs['xMax'].value
        yMin = self.calibArgs['yMin'].value
        yMax = self.calibArgs['yMax'].value
        cannyMin = self.calibArgs['cannyMin'].value
        cannyMax = self.calibArgs['cannyMax'].value

        img8 = (img/256).astype('uint8')
        ret, thresh = cv.threshold(img8,threshValMin,threshValMax,cv.THRESH_BINARY_INV)
        cropped = np.zeros(thresh.shape, thresh.dtype)
        cropped[yMin:yMax, xMin:xMax] = thresh[yMin:yMax, xMin:xMax]


        # Canny Edge detection
        canny = cv.Canny(cropped, cannyMin, cannyMax)
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
            cv.circle(thresh,(x,y), 10, (128), 5)
            cv.circle(thresh,(x+w,y), 10, (128), 5)
            cv.circle(thresh,(x+w,y+h), 10, (128), 5)
            cv.circle(thresh,(x,y+h), 10, (128), 5)
        except (UnboundLocalError):
            pass
        debugData(thresh, 10, (128),
            TopLeft=    (x,y),
            TopRight=   (x+w,y),
            BottomRight=(x+w,y+h),
            BottomLeft= (x,y+h),
            # ThresholdMin=threshValMin,
            # ThresholdMax=threshValMax,
            # xMin=xMin,
            # xMax=xMax,
            # yMin=yMin,
            # yMax=yMax,
            # cannyMin=cannyMin,
            # cannyMax=cannyMax
            **self.calibArgs)
        cv.imshow(f'{self.name}_filter', thresh)
        self.rect = rect(x,y,w,h)

    def processImage(self, input: cv.Mat):
        if self.method == calibrationMethod.THRESH_METHOD:
            self.threshMethod(input)
        if self.method == calibrationMethod.DEPTH_THRESH_METHOD:
            self.depthThreshMethod(input)


    def callback(self,data):
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
        # cv.imshow(self.name, cv_image)
        

class Calibrator():
    @property
    def error(self):
        return self.currentVal

    @property
    def lastError(self):
        return self.lastVal

    def __init__(self, init_tuning:float, threshold:float):
        self.crawlingSpeed = init_tuning
        self.threshold = threshold
        self.currentVal = 0
        self.lastVal = 0
        self.output = 0

    def compute(self, currentVal:float):
        """
        @brief Compute one iteration of the calibration algorythm
        @returns True if the threshold has been reached
        @param currentVal new process value
        """
        self.currentVal = currentVal # Update process value

        if (abs(self.error) < self.threshold): # Threshold reached
            return True
        if (self.lastError > 0) == (self.error > 0): # Has not overshot
            # If last attempt made the error bigger, we are going the wrong way, flip the crawling direction
            self.crawlingSpeed = self.crawlingSpeed if abs(self.lastError) - abs(self.error) > 0 else -self.crawlingSpeed
        else: # Overshot
            self.crawlingSpeed *= -0.8 # Reverse and reduce crawling speed
        
        self.output += self.crawlingSpeed # Update output
        self.lastVal = currentVal # Update last value
        return False # Threshold not reached

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

def colorCB(color_msg: Image):
    try:
        # image = bridge.imgmsg_to_cv2(color_msg, 'bgr8')
        # cv.imshow('rawcol', image)
        pass
    except CvBridgeError as err:
        ros.logerr(f'CvBridge error: {err}')
    key = cv.waitKeyEx(1) # Read key
    if key & 0xFF == ord('q'): # Quit
        raise ros.ROSInterruptException()


def edgeDetect(frame: cv.Mat):
    output = cv.Canny(frame, colorCannyMin, colorCannyMin*2)
    return output

def contourDetect(canny: cv.Mat, hierarchy: cv.Mat, contours: cv.Mat):
    return cv.findContours(canny, cv.RETR_TREE, cv.CHAIN_APPROX_TC89_KCOS)

def drawContours(inout: cv.Mat, hierarchy: cv.Mat, contours: cv.Mat):
    rng = Random()
    for i in range(0, len(contours)):
        rgb = [rng.randint(0, 256) for ii in range(0, 3)]
        cv.drawContours(inout, contours, i, rgb, 3, cv.LINE_8, hierarchy, 1)

def processColor(inout: cv.Mat, cannyMin: int, cannyMax: int, threshParam: int):
    # canny = edgeDetect(inout)
    # hierarchy, contours = contourDetect(canny)
    # drawContours(inout, hierarchy, contours)
    gray = cv.cvtColor(inout, cv.COLOR_BGR2GRAY)

    ret, thresh = cv.threshold(gray,threshParam,255,cv.THRESH_BINARY_INV)
    cv.imshow("filter", thresh)

    # Canny Edge detection
    canny = cv.Canny(thresh, cannyMin, cannyMax)
    # cv.addText(canny, f'Threshold:{tresh}', [0,0],  "FONT_HERSHEY_SIMPLEX")
    cv.imshow("canny", canny)

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
        cv.circle(inout,(x,y), 10, (255,255,0), 5)
        cv.circle(inout,(x+w,y), 10, (255,255,0), 5)
        cv.circle(inout,(x+w,y+h), 10, (255,255,0), 5)
        cv.circle(inout,(x,y+h), 10, (255,255,0), 5)
    except (UnboundLocalError):
        pass
    debugData(inout, 20, CannyMin=colorCannyMin,
        CannyMax=colorCannyMax,
        Threshold=colorThresh,
        TopLeft=    (x,y),
        TopRight=   (x+w,y),
        BottomRight=(x+w,y+h),
        BottomLeft= (x,y+h))
    return x, y, w, h




def setupNode():
    ros.init_node('calibration_node', anonymous=True)
    
def getScore(x1:float, y1:float, w1:float, h1:float,
             x2:float, y2:float, w2:float, h2:float):
    avgOffset = ((x1+w1/2)-(x2+w2/2),(y1+h1/2)-(y2+h2/2))
    if w2 == 0 or h2 == 0:
        avgScale = 1
    else:
        avgScale = (w1/w2+h1/h2)/2
    return avgScale, avgOffset

xCalibrate = Calibrator(0.4, 3)
yCalibrate = Calibrator(0.4, 3)

def calibrate(xScore:float, yScore:float):
    return xCalibrate.compute(xScore) and yCalibrate.compute(yScore)

def setTransform(x:float, y:float):
    pass

def simulateInit():
    cap = cv.VideoCapture(0)
    while not cap.isOpened(): # Open the webcam (test purposes only)
        cap = cv.VideoCapture(0)
        if cv.waitKey(1) & 0xFF == ord('q'):    
            raise ros.ROSInterruptException()
    return cap

def simulate(lastKey: int, cap: cv.VideoCapture):
    ret, frame = cap.read() # Read webcam frame (test only)
    if frame is not None:
        f = frame.copy()
        x1, y1, w1, h1 = processColor(f, colorCannyMin, colorCannyMax, colorThresh)
        # x2, y2, w2, h2 = processColor(f, colorCannyMin, colorCannyMax, colorThresh)
        x2 = int(x1 + 40 + xCalibrate.output * 1000)
        y2 = int(y1 - 34 + yCalibrate.output * 1000)
        cv.circle(f,(x2,y2), 10, (255,0,0), 3)
        cv.circle(f,(x2+w1,y2), 10, (255,0,0), 3)
        cv.circle(f,(x2+w1,y2+h1), 10, (255,0,0), 3)
        cv.circle(f,(x2,y2+h1), 10, (255,0,0), 3)
        # scaleOff, (x, y) = getScore(x1,y1,w1,h1,x2,y2,w2,h2)
        scaleOff, (x, y) = getScore(x1,y1,w1,h1,x2,y2,w1,h1)
        cv.imshow('frame', f)

    key = cv.waitKeyEx(1) # Read key
    if key & 0xFF == ord('q'): # Quit
        raise ros.ROSInterruptException()
    if key & 0xFF == ord('c'): # Hold to calibrate
        calibrate(x,y)
    if key == 65362 and lastKey == -1: # up arrow
        colorCannyMin += threshInc
    if key == 65364 and lastKey == -1: # down arrow
        colorCannyMin -= threshInc
    if key == 65361 and lastKey == -1: # right arrow
        colorCannyMax += threshInc
    if key == 65363 and lastKey == -1: # left arrow
        colorCannyMax -= threshInc
    if key & 0xFF  == ord('w'): 
        colorThresh += threshInc
    if key & 0xFF  == ord('s'): 
        colorThresh -= threshInc

def simulateEnd(cap: cv.VideoCapture):
    cap.release()

def oldMain():
    try:
        setupNode() # Setup the ros node
        ros.loginfo('hi') 
        mytest() # Test the calibration algorithm
        cv.namedWindow('rawcol', cv.WINDOW_AUTOSIZE) # Create a named window
        subColor = ros.Subscriber('/camera/color/image_raw', Image, colorCB) # Open the color stream
        
        while not ros.is_shutdown():
            ros.spin()
    except ros.ROSInterruptException:
        cv.destroyAllWindows()
    except KeyboardInterrupt:
        cv.destroyAllWindows()
    
async def colorStart():
    colorProcessor = imageProcessor('/camera/color/image_raw', 'bgr8', calibrationMethod.THRESH_METHOD, thresh=adjustParam(190, ord('w'), ord('s'), 5))

async def depthStart():
    depthProcessor = imageProcessor('/camera/depth/image_raw', '16UC1', calibrationMethod.DEPTH_THRESH_METHOD,
    threshMin=adjustParam(0, ord('e'), ord('d'), 1),
    threshMax=adjustParam(255, ord('r'), ord('f'), 1),
    xMin=adjustParam(0,ord('x'),ord('z')),
    xMax=adjustParam(300,ord('v'),ord('c')),
    yMin=adjustParam(0,ord('t'),ord('g')),
    yMax=adjustParam(300,ord('y'),ord('h')),
    cannyMin=adjustParam(50,ord('u'),ord('j')),
    cannyMax=adjustParam(200,ord('i'),ord('k')))


def main():
    ros.init_node('image_converter', anonymous=True)
    try:
        nodeType = ros.get_param('/calibrationColor/start_type')
    except:
        try:
            nodeType = ros.get_param('/calibrationDepth/start_type')
        except:
            pass
    
    if nodeType == 'color':
        ros.loginfo('Starting color')
        colorProcessor = imageProcessor('/camera/color/image_raw', 'bgr8', calibrationMethod.THRESH_METHOD, thresh=adjustParam(190, ord('w'), ord('s'), 5))
        ros.loginfo('Started color')
    if nodeType == 'depth':
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
