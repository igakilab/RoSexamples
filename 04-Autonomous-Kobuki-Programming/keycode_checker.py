#!/usr/bin/env python
from __future__ import print_function
from getch import getch


 
if __name__ == '__main__':
    
    while True:
        key = getch()
        print(ord(key))
        if ord(key) == 3:#Ctr+C
            raise KeyboardInterrupt
