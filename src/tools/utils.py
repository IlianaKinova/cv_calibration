from math import isclose
from numbers import Rational
from typing import Dict, Tuple
import cv2 as cv
import Xlib
from ewmh import EWMH
import numpy as np


def debugData(im:cv.Mat, height:int, color, **kwargs):
    font = cv.FONT_HERSHEY_SIMPLEX # get font
    scale = cv.getFontScaleFromHeight(font, height) # get scale from pixel height
    for i, (key, val) in enumerate(kwargs.items()): # get keywords from params plus index
        pos = (10,10 + (height + 10) * (i + 1)) # position of bottom left corner of text
        text = f'{key}: {val}' # format the keyvalue to <key>: <val>
        cv.putText(im, text, pos, font, scale, color, 2, cv.LINE_AA) # write text to image

def boxFromWindow(window):
    data = window.get_geometry()
    return {'top':int(data.y), 'left':int(data.x), 'width':int(data.width), 'height':int(data.height)}

def rvizRemoveBorders(box):
    titleBorder = 23
    outsideBorder = 1
    return {
        'top':int(box['top'] + titleBorder),
        'left':int(box['left'] + outsideBorder), 
        'width':int(box['width'] - (2 * outsideBorder)), 
        'height':int(box['height'] - (titleBorder + outsideBorder))}

def aspectRatioCorrectedRect(x:float, y:float, w:float, h:float):
    ratio = w/h
    endRatio = 16/9
    if endRatio > ratio:
        endw = w
        endh = w / endRatio
    else:
        endw = h * endRatio
        endh = h
    endx = x + w/2 - endw/2
    endy = y + h/2 - endh/2
    print(f'Before:{ratio} Ratio: {endw/endh}')
    return {'x':endx, 'y':endy, 'w':endw, 'h':endh, 'r':endRatio}

def testCase(input:Dict[str,float], expected:Dict[str,float]):
    expected.update(r=16/9)
    for (key, val) in input.items():
        if not isclose(val, expected[key], abs_tol=0.001):
            print(f'Expected {key} to be {expected[key]}, but found {val}')
        

def testAspectCorrection():
    testCase(aspectRatioCorrectedRect(2209,165,706,676), {'x':2209, 'y':304.4375, 'w':706, 'h':397.125})
    testCase(aspectRatioCorrectedRect(2973,260,662,1000), {'x':2973, 'y':573.8125, 'w':662, 'h':372.375})

def getColorStandardizedRect(box):
    ratio = float(box['width'])/box['height']
    if (16.0/9.0) > ratio:
        width = float(box['width'])
        height = float(box['width']) / (16.0/9.0)
    else:
        width = float(box['height']) * (16.0/9.0)
        height = float(box['height'])
    x = box['left'] + float(box['width'])/2.0 - float(width)/2.0
    y = box['top'] + float(box['height'])/2.0 - float(height)/2.0
    print(f'Before:{ratio} Ratio: {width/height}')
    return {'top':int(round(y)),'left':int(round(x)),'width':int(round(width)),'height':int(round(height))}
    

    