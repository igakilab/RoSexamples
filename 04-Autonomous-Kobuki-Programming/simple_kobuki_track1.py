#!/usr/bin/env python
from __future__ import print_function
import os
import sys
import rospy
import time
import curses
from getch import getch
from kobuki_msgs.msg import BumperEvent,MotorPower
from geometry_msgs.msg import Twist
import tf
from std_msgs.msg import Empty
from nav_msgs.msg import Odometry
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import numpy
from numpy.random import randint
import kobuki_status as ks

class SimpleKobuki:
    def __init__(self,scr):
        self.frame = 0
        self.scr = scr
        self.bumper_sub = rospy.Subscriber("mobile_base/events/bumper", BumperEvent, self.bumper_cb)
        self.power_cmd_pub = rospy.Publisher("mobile_base/commands/motor_power", MotorPower,queue_size=10)
        self.vel_cmd_pub = rospy.Publisher("mobile_base/commands/velocity",Twist,queue_size=10)
        self.vel_cmd = Twist()
        self.odm_reset_pub = rospy.Publisher("mobile_base/commands/reset_odometry",Empty,queue_size=10)
        self.odom_sub = rospy.Subscriber("odom",Odometry, self.odom_cb)

        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/cv_camera/image_raw",Image,self.image_cb)
        self.image_pub = rospy.Publisher("/image_converter/output_video",Image, queue_size=10)
        self.status = ks.KobukiStatus()

    def bumper_cb(self, data):
        self.scr.move(1,0)
        self.scr.clrtoeol()
        if data.state == BumperEvent.PRESSED:
            self.status.bumped = True
            self.scr.addstr(1,0,"PRESSED")
        elif data.state == BumperEvent.RELEASED:
            self.scr.addstr(1,0,"RELEASED")
        else:
            self.scr.addstr(1,0,"Bumper Unknown event")

    def odom_cb(self, msg):
        self.scr.move(0,0)
        self.scr.clrtoeol()
        self._x = msg.pose.pose.position.x
        self._y = msg.pose.pose.position.y
        # Convert quaternions to Euler angles.
        (roll, pitch, yaw) = tf.transformations.euler_from_quaternion([msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w])
        self.scr.addstr(0,0,"Odom("+str(self._x)+", "+str(self._y)+", "+str(yaw))

    def draw_contours(self, cv_image, lower, upper,cname):
        self.scr.move(2,0)
        self.scr.clrtoeol()

        # Convert BGR to HSV
        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

        # Threshold the HSV image to get only skin colors
        hueMat = cv2.inRange(hsv, lower, upper)
        kernel = numpy.ones((5,5),numpy.uint8)

        hueMat = cv2.erode(hueMat,kernel,iterations = 3)
        hueMat = cv2.dilate(hueMat,kernel,iterations = 6)
        hueMat = cv2.erode(hueMat,kernel,iterations = 3)

        # Bitwise-AND mask and original image
        res = cv2.bitwise_and(cv_image,cv_image, mask= hueMat)
        contours, hierarchy = cv2.findContours(hueMat, cv2.RETR_CCOMP, cv2.CHAIN_APPROX_SIMPLE)
        color = [100,100,100]
        cv2.drawContours(res, contours, -1, color, 3)#draw all contours

        for cont in contours:
            M = cv2.moments(cont)
            cx = int(M['m10']/M['m00'])
            cy = int(M['m01']/M['m00'])
            self.scr.addstr(2,0,"(cx,cy)=(" + str(cx) + "," + str(cy) + ")"+"color:"+cname+" ex_green:"+str(self.status.exist_green))
            x,y,w,h = cv2.boundingRect(cont)
            cv2.rectangle(cv_image,(x,y),(x+w,y+h),(0,0,255),5)
            
            if cname=="green" and cv2.contourArea(cont) > 20000:
                self.status.exist_green = 0
                if cx < 240:
                    self.status.direction = 180
                elif cx > 400:
                    self.status.direction = 0
                else:
                    self.status.direction = 90
            elif self.status.exist_green < -60: # green is not detected meanwhile
                self.status.direction = 0
            else:
                self.status.exist_green = self.status.exist_green - 1
        
        return cv_image

    def image_cb(self, data):

        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)
        #blue color
        blower = numpy.array([95, 200, 200], dtype = "uint8")
        bupper = numpy.array([115, 255, 255], dtype = "uint8")

        #green color
        glower = numpy.array([70, 200, 100], dtype = "uint8")
        gupper = numpy.array([90, 255, 200], dtype = "uint8")

        #yellow color
        ylower = numpy.array([20, 200, 100], dtype = "uint8")
        yupper = numpy.array([40, 255, 255], dtype = "uint8")

        cv_image = self.draw_contours(cv_image,blower,bupper,"blue")
        cv_image = self.draw_contours(cv_image,ylower,yupper,"yellow")
        cv_image = self.draw_contours(cv_image,glower,gupper,"green")

        cv2.imshow('original',cv_image)

        cv2.waitKey(3)
        try:
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
        except CvBridgeError as e:
            print(e)

    def kobuki_move(self):
        if self.status.power == False:
            self.power_cmd_pub.publish(MotorPower(MotorPower.OFF))
            return
        else:
            self.power_cmd_pub.publish(MotorPower(MotorPower.ON))
            
        if self.status.bumped == True:
            self.vel_cmd.linear.x = 0
            self.vel_cmd.angular.z = 0
            self.status.bumped = False
            self.status.power = False
        elif self.status.direction == 90:
            self.vel_cmd.linear.x = 0.1
            self.vel_cmd.angular.z = 0
        elif self.status.direction == 0:
            self.vel_cmd.linear.x = 0
            self.vel_cmd.angular.z = -0.4
        elif self.status.direction == 270:
            self.vel_cmd.linear.x = -0.1
            self.vel_cmd.angular.z = 0
        elif self.status.direction == 180:
            self.vel_cmd.linear.x = 0
            self.vel_cmd.angular.z = 0.4
        else:
            self.vel_cmd.linear.x = 0
            self.vel_cmd.angular.z = 0

        self.vel_cmd_pub.publish(self.vel_cmd)
        
    def waitConnection(self):
        while self.power_cmd_pub.get_num_connections() < 1:
            print("Waiting...")
            time.sleep(1)
        print("Connected")

    def spin(self):
        self.frame = self.frame+1
        key = self.scr.getch()
        message = ""
        if key in {83,115}: # S or s
            message = "START"
            self.status.power = True
        elif key in {81,113}: #Q or q
            message = "QUIT"
            self.status.power = False
        elif key in {82,114}: #R or r
            message = "ODOMETRY_RESET"
            self.odm_reset_pub.publish(Empty())
        elif key == 65: #UP
            message = "UP"
            self.status.direction = 90
        elif key == 66: #DOWN
            message = "DOWN"
            self.status.direction = 270
        elif key == 67: #RIGHT
            message = "RIGHT"
            self.status.direction = 0
        elif key == 68: #LEFT 
            message = "LEFT"
            self.status.direction = 180
        elif key == 3:#Ctr+C
            raise KeyboardInterrupt

        if message:
            message = message + " is pressed "
        self.scr.move(3,0)
        self.scr.clrtoeol()
        self.scr.addstr(3,0,message + "linear.x = "+str(self.vel_cmd.linear.x)+" angular.z = "+str(self.vel_cmd.angular.z)+" frame " + str(self.frame))
        self.kobuki_move()

def main(args):
    rospy.init_node("SimpleKobuki_View", anonymous=True)
    stdscr = curses.initscr()
    stdscr.nodelay(1)
    curses.noecho()
    kobuki = SimpleKobuki(stdscr)
    kobuki.waitConnection()
    r = rospy.Rate(20)
    try:
        while not rospy.is_shutdown():
            kobuki.spin()
            stdscr.refresh()
            r.sleep()
    except KeyboardInterrupt:
        print("\r\033[K"+"Shutting down")
    #Clean up curses.
    curses.nocbreak()
    stdscr.keypad(0)
    curses.echo()
    curses.endwin()
 
if __name__ == '__main__':
    os.system("clear")
    main(sys.argv)
