from __future__ import print_function

import sys
import cv2
import numpy
from numpy.random import randint

WINNAME = "OpenCV Sample 04"
WIDTH = 640
HEIGHT = 480

#blue color
#lower = numpy.array([110, 100, 100], dtype = "uint8")
#upper = numpy.array([130, 255, 255], dtype = "uint8")

#skin color
lower = numpy.array([0, 48, 80], dtype = "uint8")
upper = numpy.array([20, 255, 255], dtype = "uint8")

if __name__ == '__main__':
    #cv2.namedWindow(WINNAME)

    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        sys.exit(1)

    while True:
        # Take each frame
        _, frame = cap.read()
        # Convert BGR to HSV
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # Threshold the HSV image to get only skin colors
        hueMat = cv2.inRange(hsv, lower, upper)
        kernel = numpy.ones((5,5),numpy.uint8)

        hueMat = cv2.erode(hueMat,kernel,iterations = 3)
        hueMat = cv2.dilate(hueMat,kernel,iterations = 6)
        hueMat = cv2.erode(hueMat,kernel,iterations = 3)

        # Bitwise-AND mask and original image
        res = cv2.bitwise_and(frame,frame, mask= hueMat)
        contours, hierarchy = cv2.findContours(hueMat, cv2.RETR_CCOMP, cv2.CHAIN_APPROX_SIMPLE)
        color = [randint(256) for _ in range(3)]
        cv2.drawContours(res, contours, -1, color, 3)#draw all contours

        for cont in contours:
            M = cv2.moments(cont)
            cx = int(M['m10']/M['m00'])
            cy = int(M['m01']/M['m00'])
            print("(cx,cy)=(" + str(cx) + "," + str(cy) + ")")
            cv2.circle(frame, (cx,cy),50,(0,0,255),10)
        cv2.imshow('frame',frame)
        cv2.imshow('mask',hueMat)
        cv2.imshow('res',res)

        key = cv2.waitKey(1)
        if key%256 == ord('q'):
            break
