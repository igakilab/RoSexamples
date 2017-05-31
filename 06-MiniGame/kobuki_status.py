#!/usr/bin/env python
from __future__ import print_function

class KobukiStatus:
    def __init__(self):
        self.power = False
        self.blueball = False #If KOBUKI found the blue ball, this status becomes True.