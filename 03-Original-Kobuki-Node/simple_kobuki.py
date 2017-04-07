#!/usr/bin/env python
from __future__ import print_function
import sys
import rospy
import curses
from kobuki_msgs.msg import BumperEvent
from std_msgs.msg import String
 
class SimpleKobuki:
    def __init__(self):
        self.bumper_sub = rospy.Subscriber("mobile_base/events/bumper", BumperEvent, self.bumper_cb)
 
    def bumper_cb(self, data):
        if data.state == BumperEvent.PRESSED:
            print("PRESSED")
        elif data.state == BumperEvent.RELEASED:
            print("RELEASED")
        else:
            print("Bumper Unknown event")
 
def main(args):
    stdscr = curses.initscr()
    curses.cbreak()
    curses.noecho()
    stdscr.keypad(True)
    stdscr.timeout(10)
 
    kobuki = SimpleKobuki()
    rospy.init_node("SimpleKobuki", anonymous=True)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
 
if __name__ == '__main__':
    main(sys.argv)
