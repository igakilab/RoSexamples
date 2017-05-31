#!/usr/bin/env python
from __future__ import print_function

class KobukiStatus:
    def __init__(self):
        self.power = False
        self.blueball = False #If KOBUKI found the blue ball, this status becomes True.
        self.blueballcount = 0
        self.blueballx = 0 #center point(cx,cy) of the detected blue ball.
        self.bluebally = 0