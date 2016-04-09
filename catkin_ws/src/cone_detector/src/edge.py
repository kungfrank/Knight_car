import numpy as np
import cv2

im = cv2.imread('image.png')
hsv_img = cv2.cvtColor(im, cv2.COLOR_BGR2HSV)
COLOR_MIN = np.array([0, 80, 80],np.uint8)
COLOR_MAX = np.array([22, 255, 255],np.uint8)
frame_threshed = cv2.inRange(hsv_img, COLOR_MIN, COLOR_MAX)
imgray = frame_threshed
ret,thresh = cv2.threshold(frame_threshed,22,255,0)
contours, hierarchy = cv2.findContours(thresh,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)

# Find the index of the largest contour
areas = [cv2.contourArea(c) for c in contours]
max_index = np.argmax(areas)
cnt = contours[max_index]
#for cnt in contours[:]:
x,y,w,h = cv2.boundingRect(cnt)
cv2.rectangle(im,(x,y),(x+w,y+h),(0,255,0),2)
# approximate the contour
peri = cv2.arcLength(cnt, True)
approx = cv2.approxPolyDP(cnt,   peri, True)

# if our approximated contour has four points, then
# we can assume that we have found our screen
if len(approx) == 3:
        screenCnt = approx
        cv2.rectangle(im,(x,y),(x+w,y+h),(0,255,0),20)
cv2.imshow("Show",im)
cv2.waitKey()
cv2.destroyAllWindows()
