from __future__ import print_function

import sys
import cv2
import numpy

WINNAME = "OpenCV Sample 01"
WIDTH = 640
HEIGHT = 480

def run():
    img = numpy.zeros((HEIGHT, WIDTH, 3))
    for r in range(256):
        for g in range(256):
            for b in range(256):
                img[...] = (b, g, r)
                print("(r, g, b) = ({}, {}, {})".format(r, g, b), file=sys.stderr)
                cv2.imshow(WINNAME, img)
                print(img)

                key = cv2.waitKey(1)
                if key == ord('q'):
                    return

if __name__ == '__main__':
    cv2.namedWindow(WINNAME)
    run()
