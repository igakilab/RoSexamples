from __future__ import print_function

import sys
import cv2
import numpy
from numpy.random import randint

WINNAME = "OpenCV Sample 04"
WIDTH = 640
HEIGHT = 480

if __name__ == '__main__':
    cv2.namedWindow(WINNAME)

    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        sys.exit(1)

    while True:
        success, frame = cap.read()
        frame.resize((HEIGHT, WIDTH, 3))

        image = numpy.copy(frame)
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        hueMat = cv2.inRange(hsv, (0, 50, 50), (15, 255, 255))
        kernel = numpy.ones((3,3),numpy.uint8)

        hueMat = cv2.erode(hueMat,kernel,iterations = 3)
        hueMat = cv2.dilate(hueMat,kernel,iterations = 6)
        hueMat = cv2.erode(hueMat,kernel,iterations = 3)

        image[hueMat == 255] = (0, 255, 0)

        contours, hierarchy = cv2.findContours(hueMat, cv2.RETR_CCOMP, cv2.CHAIN_APPROX_SIMPLE)

        color = [randint(256) for _ in range(3)]
        cv2.drawContours(image, contours, -1, color, 3)

        cv2.imshow(WINNAME, image)

        key = cv2.waitKey(1)
        if key%256 == ord('q'):
            break
