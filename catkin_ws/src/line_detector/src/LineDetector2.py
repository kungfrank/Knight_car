import numpy as np
import cv2
import sys

class LineDetector(object):
	def __init__(self):
		self.color_white1 = np.array([150, 150, 150])
		self.color_white2 = np.array([255, 255, 255])
		self.color_yellow1 = np.array([0, 150, 150])
		self.color_yellow2 = np.array([150, 255, 255]) 
		self.color_red1 = np.array([0, 0, 150])
		self.color_red2 = np.array([50, 50, 255]) 

	def __colorFilter(self, bgr, color):
		if color == 'white':
			color1 = self.color_white1
			color2 = self.color_white2		
		elif color == 'yellow':
			color1 = self.color_yellow1
			color2 = self.color_yellow2		
		elif color == 'red':
			color1 = self.color_red1
			color2 = self.color_red2		
		else:
			raise Exception('Error: Undefined color strings...')
		
		# threshold lanes by color
		lane = cv2.inRange(bgr, color1, color2)
		# binary image processing
		kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE,(3, 3))
		lane = cv2.erode(lane, kernel)
		kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE,(5, 5))
		lane = cv2.dilate(lane, kernel)
		return lane

	def __findEdge(self, gray):	
		edges = cv2.Canny(gray, 10, 30, apertureSize = 3)
		return edges

	def __HoughLine(self, edge, bgr):
		lines = cv2.HoughLinesP(edge, 1, np.pi/180, 100, np.empty(1), minLineLength=100, maxLineGap=20)
		if lines is not None:
			lines = lines[0]
		return lines

	def detectLines(self, bgr, color):
		lane = self.__colorFilter(bgr, color)
		edges = self.__findEdge(lane)
		lines = self.__HoughLine(edges, bgr)	
		return lines
	
	def drawLines(self, bgr, lines, paint):
		if lines is not None:
			for x1,y1,x2,y2 in lines:
				cv2.line(bgr, (x1,y1), (x2,y2), paint, 3)

"""
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
	# read video from file or camera
	if len(sys.argv)==2:
		cap = cv2.VideoCapture(sys.argv[1])
		if not cap.isOpened():
			print 'Error opening file...'
			return -1
	elif len(sys.argv)==1:
		cap = cv2.VideoCapture(0)
		if not cap.isOpened():
			print 'Error opening camera...'
			return -1
	else:
		return -1

	while True:
		ret, bgr = cap.read()
		if not ret:
			print 'No frames grabbed...'
			break

		detector = LineDetector()

		lines_white = detector.detectLines(bgr, 'white')
		lines_yellow = detector.detectLines(bgr, 'yellow')
		lines_red = detector.detectLines(bgr, 'red')

		detector.drawLines(bgr, lines_white, (0,0,0))
		detector.drawLines(bgr, lines_yellow, (255,0,0))
		detector.drawLines(bgr, lines_red, (0,255,0))

		cv2.imshow('Line Detector', bgr)

		cv2.waitKey(30)

if __name__ == '__main__':
	_main()	
