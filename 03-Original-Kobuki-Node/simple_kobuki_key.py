#!/usr/bin/env python
from __future__ import print_function
import os
import sys
import rospy
import time
from getch import getch
from kobuki_msgs.msg import BumperEvent,MotorPower
from geometry_msgs.msg import Twist
 
class SimpleKobuki:
    def __init__(self):
        self.bumper_sub = rospy.Subscriber("mobile_base/events/bumper", BumperEvent, self.bumper_cb)
        self.power_cmd_pub = rospy.Publisher("mobile_base/commands/motor_power", MotorPower,queue_size=10)
        self.vel_cmd_pub = rospy.Publisher("mobile_base/commands/velocity",Twist,queue_size=10)
        self.vel_cmd = Twist()

    def bumper_cb(self, data):
        sys.stdout.flush()
        if data.state == BumperEvent.PRESSED:
            sys.stdout.write("PRESSED \r")
        elif data.state == BumperEvent.RELEASED:
            sys.stdout.write("RELEASED\r")
        else:
            sys.stdout.write("Bumper Unknown event\r")

    def waitConnection(self):
        while self.power_cmd_pub.get_num_connections() < 1:
            print("Waiting...")
            time.sleep(1)
        print("Connected")

    def spin(self):
        key = ord(getch())
        message = ""
        if key in {83,115}: # S or s
            message = "START"
            self.power_cmd_pub.publish(MotorPower(MotorPower.ON))
        elif key in {81,113}: #Q or q
            message = "QUIT"
            self.power_cmd_pub.publish(MotorPower(MotorPower.OFF))
        elif key == 65: #UP
            message = "UP"
            self.vel_cmd.linear.x += 0.05
        elif key == 66: #DOWN
            message = "DOWN"
            self.vel_cmd.linear.x -= 0.05
        elif key == 67: #RIGHT
            message = "RIGHT"
            self.vel_cmd.angular.z -= 0.4
        elif key == 68: #LEFT 
            message = "LEFT"
            self.vel_cmd.angular.z += 0.4
        elif key == 3:#Ctr+C
            raise KeyboardInterrupt
        else:
            pass

        if message:
            message = message + " is pressed "

        self.vel_cmd_pub.publish(self.vel_cmd)
        sys.stdout.write(message + "linear.x = "+str(self.vel_cmd.linear.x)+" angular.z = "+str(self.vel_cmd.angular.z)+"                  \r")

def main(args):
    rospy.init_node("SimpleKobuki_Key", anonymous=True) 
    kobuki = SimpleKobuki()
    kobuki.waitConnection()
    try:
        while not rospy.is_shutdown():
            kobuki.spin()
    except KeyboardInterrupt:
        print("Shutting down                               ")
 
if __name__ == '__main__':
    os.system("clear")
    main(sys.argv)
