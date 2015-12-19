import numpy as np
import cv2
import sys

class LineDetector(object):
	def __init__(self):
		self.yuv_white1 = np.array([150, 100, 100])
		self.yuv_white2 = np.array([255, 160, 160])
		self.yuv_yellow1 = np.array([100, 150, 0])
		self.yuv_yellow2 = np.array([200, 200, 50]) 
		self.yuv_red1 = np.array([0, 150, 110])
		self.yuv_red2 = np.array([100, 255, 140]) 

	def __colorFilter(self, bgr, color):
		# transform into YUV color space
		yuv = cv2.cvtColor(bgr, cv2.COLOR_BGR2YUV)
		if color == 'white':
			yuv_color1 = self.yuv_white1
			yuv_color2 = self.yuv_white2		
		elif color == 'yellow':
			yuv_color1 = self.yuv_yellow1
			yuv_color2 = self.yuv_yellow2		
		elif color == 'red':
			yuv_color1 = self.yuv_red1
			yuv_color2 = self.yuv_red2		
		else:
			raise Exception('Error: Undefined color strings...')
		
		# threshold lanes by color in YUV space
		lane = cv2.inRange(yuv, yuv_color1, yuv_color2)

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
		lines = cv2.HoughLinesP(edge, 1, np.pi/180, 100, np.empty(1), minLineLength=30, maxLineGap=20)
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

	def getLane(self, bgr, color):
		lane = self.__colorFilter(bgr, color)
		return lane

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

		# lane = detector.getLane(bgr, 'yellow')
		# cv2.imshow('Yellow lane', lane)

if __name__ == '__main__':
	_main()	
