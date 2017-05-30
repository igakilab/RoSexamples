#!/usr/bin/env python
from __future__ import print_function
import os
import sys
import rospy
import time
import curses
from kobuki_msgs.msg import BumperEvent, MotorPower
from geometry_msgs.msg import Twist

class SimpleKobuki:
    def __init__(self,scr): #constructor
        self.scr = scr #for curses
        self.bumper_sub = rospy.Subscriber("mobile_base/events/bumper", BumperEvent, self.bumper_cb)
        self.power_cmd_pub = rospy.Publisher("mobile_base/commands/motor_power", MotorPower,queue_size=10)
        self.vel_cmd_pub = rospy.Publisher("mobile_base/commands/velocity",Twist,queue_size=10)
        self.vel_cmd = Twist()
        
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
            self.power_cmd_pub.publish(MotorPower(MotorPower.OFF))
        elif bumper.state == BumperEvent.RELEASED:
            self.scr.addstr(1,0,"RELEASED")
        else:
            self.scr.addstr(1,0,"Bumper Unknown event")
            
    def spin(self):
        key = self.scr.getch()
        message = ""
        if key in {83,115}: # S or s
            message = "START"
            self.power_cmd_pub.publish(MotorPower(MotorPower.ON))
            self.vel_cmd.linear.x = 0.1
            self.vel_cmd.angular.z = 0
        elif key in {81,113}: #Q or q
            message = "QUIT"
            self.power_cmd_pub.publish(MotorPower(MotorPower.OFF))
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
        self.vel_cmd_pub.publish(self.vel_cmd)

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
