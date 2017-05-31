#!/usr/bin/env python
from __future__ import print_function
import os
import sys
import rospy
import time
import curses
from kobuki_msgs.msg import BumperEvent, MotorPower
from geometry_msgs.msg import Twist
import kobuki_status as ks
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import numpy

class SimpleKobuki:
    def __init__(self,scr): #constructor
        self.scr = scr #for curses
        self.bumper_sub = rospy.Subscriber("mobile_base/events/bumper", BumperEvent, self.bumper_cb)
        self.power_cmd_pub = rospy.Publisher("mobile_base/commands/motor_power", MotorPower,queue_size=10)
        self.vel_cmd_pub = rospy.Publisher("mobile_base/commands/velocity",Twist,queue_size=10)
        self.vel_cmd = Twist()
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/cv_camera/image_raw",Image,self.image_cb)
        self.image_pub = rospy.Publisher("/image_converter/output_video",Image, queue_size=10)
        self.status = ks.KobukiStatus()
        
    def waitConnection(self):
        while self.power_cmd_pub.get_num_connections() < 1:
            print("Waiting...")
            time.sleep(1)
        print("Connected")

    def bumper_cb(self, bumper):
        self.scr.move(1,0) #1st line
        self.scr.clrtoeol() #Clear to End of Line
        if bumper.state == BumperEvent.PRESSED:
            self.scr.addstr(1,0,"PRESSED")
            self.status.power = False
        elif bumper.state == BumperEvent.RELEASED:
            self.scr.addstr(1,0,"RELEASED")
        else:
            self.scr.addstr(1,0,"Bumper Unknown event")

    def image_cb(self, rosimage):
        self.scr.move(2,0)
        self.scr.clrtoeol()

        try:#Convert ros image into open cv image
            cv_image = self.bridge.imgmsg_to_cv2(rosimage, "bgr8")
        except CvBridgeError as e:
            print(e)

        #green color
        lower = numpy.array([70, 200, 100], dtype = "uint8")
        upper = numpy.array([90, 255, 200], dtype = "uint8")

        # Convert BGR to HSV
        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

        # Threshold the HSV image to get only skin colors
        hueMat = cv2.inRange(hsv, lower, upper)
        kernel = numpy.ones((5,5),numpy.uint8)

        #Eliminate noises
        hueMat = cv2.erode(hueMat,kernel,iterations = 3)
        hueMat = cv2.dilate(hueMat,kernel,iterations = 6)
        hueMat = cv2.erode(hueMat,kernel,iterations = 3)

        #detect contours
        contours, hierarchy = cv2.findContours(hueMat, cv2.RETR_CCOMP, cv2.CHAIN_APPROX_SIMPLE)

        #draw rectangle surrunding each contour
        for cont in contours:
            M = cv2.moments(cont)
            cx = int(M['m10']/M['m00'])
            cy = int(M['m01']/M['m00'])
            self.scr.addstr(2,0,"(cx,cy)=(" + str(cx) + "," + str(cy) + ")")
            x,y,w,h = cv2.boundingRect(cont)
            cv2.rectangle(cv_image,(x,y),(x+w,y+h),(0,0,255),5)
            self.status.blueball = True

        cv2.imshow('original',cv_image)

        cv2.waitKey(3)
        try:#Convert opencv image into ros image.
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
        except CvBridgeError as e:
            print(e)
            
    def kobuki_move(self):
        if self.status.power == False:
            self.power_cmd_pub.publish(MotorPower(MotorPower.OFF))
            return
        else:
            self.power_cmd_pub.publish(MotorPower(MotorPower.ON))
        
        if self.status.blueball == True:
            self.vel_cmd.linear.x = 0
            self.vel_cmd.angular.z = 0
            self.status.blueball = False
        elif self.status.blueball == False:
            self.vel_cmd.linear.x = 0
            self.vel_cmd.angular.z = -0.4
        
        self.vel_cmd_pub.publish(self.vel_cmd)
        
    def spin(self):
        key = self.scr.getch()
        message = ""
        if key in {83,115}: # S or s
            message = "START"
            self.status.power = True
            self.vel_cmd.linear.x = 0
            self.vel_cmd.angular.z = -0.4
        elif key in {81,113}: #Q or q
            message = "QUIT"
            self.status.power = False
        elif key == 65: #UP
            message = "UP"
            self.vel_cmd.linear.x = 0.1
            self.vel_cmd.angular.z = 0
        elif key == 66: #DOWN
            message = "DOWN"
            self.vel_cmd.linear.x = -0.1
            self.vel_cmd.angular.z = 0
        elif key == 67: #RIGHT
            message = "RIGHT"
            self.vel_cmd.linear.x = 0
            self.vel_cmd.angular.z = -0.4
        elif key == 68: #LEFT 
            message = "LEFT"
            self.vel_cmd.linear.x = 0
            self.vel_cmd.angular.z = 0.4
        elif key == 3:#Ctr+C
            raise KeyboardInterrupt

        if message:
            message = message + " is pressed "
        self.scr.move(3,0)
        self.scr.clrtoeol()
        self.scr.addstr(3,0,message + "linear.x = "+str(self.vel_cmd.linear.x)+" angular.z = "+str(self.vel_cmd.angular.z))
        self.kobuki_move()

def main(args):
    rospy.init_node("minigame_key", anonymous=True)
    #init curses setting
    stdscr = curses.initscr()
    stdscr.nodelay(1)
    curses.noecho()
    
    #create SimpleKobuki Object
    kobuki = SimpleKobuki(stdscr)
    kobuki.waitConnection()
    
    #20 means 20Hz.This setting is corresponding to r.sleep()
    r = rospy.Rate(20)
    try:
        while not rospy.is_shutdown():
            kobuki.spin()
            stdscr.refresh()
            r.sleep() #waits 1/20 s
    except KeyboardInterrupt:
        print("Shutting down")
    #Clean up curses.
    curses.nocbreak()
    stdscr.keypad(0)
    curses.echo()
    curses.endwin()
 
if __name__ == '__main__':
    os.system("clear")
    main(sys.argv)
