#!/usr/bin/env python
from __future__ import print_function

class KobukiStatus:
    def __init__(self):
        self.power = False
        self.direction = 0 # relative direction 0-360
        self.bumped = False

    def print_status(self):
        print(self.start)
