
from hand_tracking import relativeMovement 
from collections import deque
from imutils.video import VideoStream
import numpy as np
import argparse
import cv2
import imutils
import time

class App:
    def __init__(self):
        self.rel = relativeMovement()

    def loop(self):
        self.rel.start()

if __name__ == "main":
    app = App()
    app.loop()