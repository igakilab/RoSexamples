#!/usr/bin/env python
from __future__ import print_function
import os
import sys
import rospy
import time
from getch import getch
from kobuki_msgs.msg import BumperEvent,MotorPower
from geometry_msgs.msg import Twist
import tf
from std_msgs.msg import Empty
from nav_msgs.msg import Odometry
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
 
class SimpleKobuki:
    def __init__(self):
        self.bumper_sub = rospy.Subscriber("mobile_base/events/bumper", BumperEvent, self.bumper_cb)
        self.power_cmd_pub = rospy.Publisher("mobile_base/commands/motor_power", MotorPower,queue_size=10)
        self.vel_cmd_pub = rospy.Publisher("mobile_base/commands/velocity",Twist,queue_size=10)
        self.vel_cmd = Twist()
        self.odm_reset_pub = rospy.Publisher("mobile_base/commands/reset_odometry",Empty,queue_size=10)
        self.odom_sub = rospy.Subscriber("odom",Odometry, self.odom_cb)

        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/cv_camera/image_raw",Image,self.image_cb)
        self.image_pub = rospy.Publisher("/image_converter/output_video",Image, queue_size=10)

    def bumper_cb(self, data):
        sys.stdout.flush()
        if data.state == BumperEvent.PRESSED:
            sys.stdout.write("\r\033[K" + "PRESSED")
        elif data.state == BumperEvent.RELEASED:
            sys.stdout.write("\r\033[K" + "RELEASED")
        else:
            sys.stdout.write("\r\033[K" + "Bumper Unknown event")

    def odom_cb(self, msg):
        self._x = msg.pose.pose.position.x
        self._y = msg.pose.pose.position.y
        # Convert quaternions to Euler angles.
        (roll, pitch, yaw) = tf.transformations.euler_from_quaternion([msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w])
        sys.stdout.write("\r" + "Odom("+str(self._x)+", "+str(self._y)+", "+str(yaw))
    
    def image_cb(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)
        
        (rows,cols,channels) =  cv_image.shape
        if cols > 60 and rows > 60:
            cv2.circle(cv_image, (50,50), 10, 255)

        cv2.imshow("Image window", cv_image)
        cv2.waitKey(3)
        try:
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
        except CvBridgeError as e:
            print(e)

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
        elif key in {82,114}: #R or r
            message = "ODOMETRY_RESET"
            self.odm_reset_pub.publish(Empty())
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
            print("\r\033[K" + message + "linear.x = "+str(self.vel_cmd.linear.x)+" angular.z = "+str(self.vel_cmd.angular.z))

def main(args):
    rospy.init_node("SimpleKobuki_Odom", anonymous=True) 
    kobuki = SimpleKobuki()
    kobuki.waitConnection()
    try:
        while not rospy.is_shutdown():
            kobuki.spin()
    except KeyboardInterrupt:
        print("\r\033[K"+"Shutting down")
 
if __name__ == '__main__':
    os.system("clear")
    main(sys.argv)
