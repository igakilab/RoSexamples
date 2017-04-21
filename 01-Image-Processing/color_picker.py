from __future__ import print_function
import numpy as np
import cv2

def mouse_event(event, x, y, flg, prm):
    if event==cv2.EVENT_LBUTTONDOWN:
	img = np.ones((128, 128, 3), np.uint8)
	avbgr = np.array([(np.uint8)(np.average(frame[y-2:y+2, x-2:x+2,0])),
			  (np.uint8)(np.average(frame[y-2:y+2, x-2:x+2,1])),
			  (np.uint8)(np.average(frame[y-2:y+2, x-2:x+2,2]))])
                
	img[:,:,0] = img[:,:,0] * avbgr[0]
	img[:,:,1] = img[:,:,1] * avbgr[1]
	img[:,:,2] = img[:,:,2] * avbgr[2]
	
	cv2.imshow('average color', img)
        
	print('bgr: '+str(img[1,1,:]))
	avhsv = cv2.cvtColor(np.array([[avbgr]], np.uint8), cv2.COLOR_BGR2HSV)
	print('hsv: '+str(avhsv[0,0,:]))
                
cap=cv2.VideoCapture(0)
cv2.namedWindow('camera capture')
cv2.setMouseCallback( 'camera capture', mouse_event )

while True:
    ret,frame=cap.read()
    cv2.imshow('camera capture', frame)
	
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
	
cap.release()
cv2.destroyAllWindows()
