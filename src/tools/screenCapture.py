from typing import Dict
import cv2 as cv
import numpy as np
from mss.exception import ScreenShotError
import rospy as ros



class screenCapture:
    def __init__(self, boundingBox:Dict[str,int]):
        from tools.constants import sct
        self.box = boundingBox
        self.last_img = None
        self.sct = sct

    def getImg(self):
        return np.array(self.last_img)

    def capture(self):
        try:
            self.last_img = self.sct.grab(self.box)
        except ScreenShotError as e:
            ros.logerr_throttle(500, f"Couldn't perform capture: {e}")
