#!/usr/bin/env python
from __future__ import print_function
import sys
import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import numpy
from numpy.random import randint
 
lower = numpy.array([0, 48, 80], dtype = "uint8")
upper = numpy.array([20, 255, 255], dtype = "uint8")

class image_converter:
    def __init__(self):
        self.image_pub = rospy.Publisher("/image_converter/output_video", Image, queue_size=10)
 
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/cv_camera/image_raw", Image, self.callback)
 
    def callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)
 
        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        hueMat = cv2.inRange(hsv, lower, upper)
        kernel = numpy.ones((3,3),numpy.uint8)
 
        hueMat = cv2.erode(hueMat,kernel,iterations = 3)
        hueMat = cv2.dilate(hueMat,kernel,iterations = 6)
        hueMat = cv2.erode(hueMat,kernel,iterations = 3)
 
        cv_image[hueMat == 255] = (0, 255, 0)
 
        contours, hierarchy = cv2.findContours(hueMat, cv2.RETR_CCOMP, cv2.CHAIN_APPROX_SIMPLE)
 
        color = [randint(256) for _ in range(3)]
        cv2.drawContours(cv_image, contours, -1, color, 3)
 
        cv2.imshow("Image window", cv_image)
        cv2.waitKey(3)
 
        try:
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
        except CvBridgeError as e:
            print(e)
 
def main(args):
    rospy.init_node('image_converter', anonymous=True)
    ic = image_converter()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()
 
if __name__ == '__main__':
    main(sys.argv)
