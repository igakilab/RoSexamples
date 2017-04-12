from __future__ import print_function

import cv2
import numpy

if __name__ == '__main__':
    blue = numpy.uint8([[[255,0,0]]]) #BGR
    hsv_blue = cv2.cvtColor(blue,cv2.COLOR_BGR2HSV)
    print(hsv_blue)
