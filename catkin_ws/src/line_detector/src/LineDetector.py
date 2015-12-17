import numpy as np
import cv2
import sys

def lineDetector(bgr):
	# Return data structure: line[0] for white, line[1] for yellow
	lines = [None] * 2

	# Color filtering: white and yellow
	lane_w = cv2.inRange(bgr, np.array([150, 150, 150]), np.array([255, 255, 255]))
	lane_y = cv2.inRange(bgr, np.array([0, 150, 150]), np.array([150, 255, 255]))
	kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE,(3, 3))
	lane_w = cv2.erode(lane_w, kernel)
	lane_y = cv2.erode(lane_y, kernel)
	kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE,(5, 5))
	lane_w = cv2.dilate(lane_w, kernel)
	lane_y = cv2.dilate(lane_y, kernel)

	# Canny edge detection
	gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
	gray = cv2.GaussianBlur(gray, ksize=(7,7), sigmaX=5.0)
	edges = cv2.Canny(gray, 10, 30, apertureSize = 3)

	# Hough Line Transform
	lane_w = cv2.bitwise_and(lane_w, edges)
	lines[0] = cv2.HoughLinesP(lane_w, 1, np.pi/180, 100, np.empty(1), minLineLength=100, maxLineGap=20)
	if lines[0] is not None:
		lines[0] = lines[0][0]
		# draw red lines around white lanes
		for x1,y1,x2,y2 in lines[0]:
			cv2.line(bgr, (x1,y1), (x2,y2), (0,0,255), 3)
		
	lane_y = cv2.bitwise_and(lane_y, edges)
	lines[1] = cv2.HoughLinesP(lane_y, 1, np.pi/180, 100, np.empty(1), minLineLength=100, maxLineGap=20)
	if lines[1] is not None:
		lines[1] = lines[1][0]
		# draw blue lines around yellow lanes
		for x1,y1,x2,y2 in lines[1]:
			cv2.line(bgr, (x1,y1), (x2,y2), (255,0,0), 3)
	
	return lines

"""
def _main():
	if len(sys.argv)!=2:
		print 'Error reading filename...'
		return -1

	cap = cv2.VideoCapture(sys.argv[1])
	if not cap.isOpened():
		print 'Error opening file...'
		return -1	
	
	while True:
		ret, bgr = cap.read()
		print ret
		if bgr.empty():
			break

		gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
		cv2.imshow('bgr',gray)
		cv2.waitKey(30)
		
		lineDetector(bgr)
		cv2.imshow('Lane Detector', bgr)
		cv2.waitKey(30)

if __name__ == '__main__':
	_main()	
"""
