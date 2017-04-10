import cv2

if __name__ == '__main__':
    WINNAME = "OpenCV Sample 00"
    cv2.namedWindow(WINNAME)

    img = cv2.imread('./image00.jpg')

    cv2.imshow(WINNAME, img)
    key = cv2.waitKey(0)
