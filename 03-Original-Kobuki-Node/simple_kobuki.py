#!/usr/bin/env python
from __future__ import print_function
import os
import sys
import rospy
from kobuki_msgs.msg import BumperEvent

class SimpleKobuki:
    def __init__(self):
        self.bumper_sub = rospy.Subscriber("mobile_base/events/bumper", BumperEvent, self.bumper_cb)
 
    def bumper_cb(self, data):
        sys.stdout.flush()
        if data.state == BumperEvent.PRESSED:
            sys.stdout.write("PRESSED \r")
        elif data.state == BumperEvent.RELEASED:
            sys.stdout.write("RELEASED\r")
        else:
            sys.stdout.write("Bumper Unknown event\r")
 
def main(args):
 
    kobuki = SimpleKobuki()
    rospy.init_node("SimpleKobuki", anonymous=True)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
 
if __name__ == '__main__':
    os.system("clear")
    main(sys.argv)
