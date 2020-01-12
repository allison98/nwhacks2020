from collections import deque
from imutils.video import VideoStream
import numpy as np
import argparse
import cv2
import imutils
import time
import serial

cap2 = cv2.VideoCapture(2)

while True:
			# grab the current frame
    ret, thirdframe = cap2.read()
    cv2.imshow("ArmCamera", thirdframe)


    key=cv2.waitKey(1) & 0xFF

			# if the 'q' key is pressed, stop the loop
    if key == ord("q"):
        break

cap2.release()

		# close all windows
cv2.destroyAllWindows()

