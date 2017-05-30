from __future__ import print_function
import sys
import cv2
import numpy

WINNAME = "Skin Detect"
WIDTH = 640
HEIGHT = 480

#skin color
#lower = numpy.array([0, 48, 80], dtype = "uint8")
#upper = numpy.array([20, 255, 255], dtype = "uint8")

#ball color
lower = numpy.array([90, 160, 100], dtype = "uint8")
upper = numpy.array([110, 230, 230], dtype = "uint8")
        
if __name__ == '__main__':
    cv2.namedWindow(WINNAME)

    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        sys.exit(1)

    while True:
        _, frame = cap.read() #Take each frame
        frame.resize((HEIGHT, WIDTH, 3))

        #Convert BGR to HSV
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        hueMat = cv2.inRange(hsv, lower, upper)
        kernel = numpy.ones((5,5),numpy.uint8)

        #eliminate noise
        hueMat = cv2.erode(hueMat,kernel,iterations = 3)
        hueMat = cv2.dilate(hueMat,kernel,iterations = 6)
        hueMat = cv2.erode(hueMat,kernel,iterations = 3)

        contours, hierarchy = cv2.findContours(hueMat, cv2.RETR_CCOMP, cv2.CHAIN_APPROX_SIMPLE)
        color = [255,0,0]

        #draw rectangle surrunding each contour
        for cont in contours:
            M = cv2.moments(cont)
            cx = int(M['m10']/M['m00'])
            cy = int(M['m01']/M['m00'])
            print("(cx,cy)=(" + str(cx) + "," + str(cy) + ")") #center point of each rectangle
            x,y,w,h = cv2.boundingRect(cont)#x,y w(width), h(height)
            cv2.rectangle(frame,(x,y),(x+w,y+h),(0,0,255),5)

        cv2.imshow(WINNAME, frame)

        key = cv2.waitKey(1)
        if key%256 == ord('q'):
            break