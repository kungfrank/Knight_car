import numpy as np
import cv2
import sys
import time

class LineDetector(object):
    def __init__(self):
        self.bgr_white1 = np.array([120, 120, 120])
        self.bgr_white2 = np.array([255, 255, 255])
        self.hsv_yellow1 = np.array([20, 150, 120])
        self.hsv_yellow2 = np.array([40, 255, 255]) 
        self.hsv_red1 = np.array([0, 120, 150])
        self.hsv_red2 = np.array([10, 255, 255]) 
        self.hsv_red3 = np.array([245, 120, 150])
        self.hsv_red4 = np.array([255, 255, 255]) 

    def __colorFilter(self, bgr, color):
        tic = time.time()
        # filter white in BGR space, yellow and red in HSV space
        # colors are hard-coded, making the code extremely ugly...could be further improved
        if color == 'white':
            lane = cv2.inRange(bgr, self.bgr_white1, self.bgr_white2)
        elif color == 'yellow':
            hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
            lane = cv2.inRange(hsv, self.hsv_yellow1, self.hsv_yellow2)
        elif color == 'red':
            hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
            lane1 = cv2.inRange(hsv, self.hsv_red1, self.hsv_red2)
            lane2 = cv2.inRange(hsv, self.hsv_red3, self.hsv_red4)
            lane = cv2.bitwise_or(lane1, lane2)
        else:
	        raise Exception('Error: Undefined color strings...')
        print 'Color thresholding:' + str(time.time()-tic)

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
        lines = cv2.HoughLinesP(edge, 1, np.pi/180, 50, np.empty(1), minLineLength=5, maxLineGap=5)
        if lines is not None:
            lines = lines[0]
        else:
            lines = []
        return lines

    def detectLines(self, bgr, color):
        lane = self.__colorFilter(bgr, color)
        edges = self.__findEdge(lane)
        lines = self.__HoughLine(edges, bgr)
        return lines

    def drawLines(self, bgr, lines, paint):
        if len(lines)>0:
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
        bgr = bgr[bgr.shape[0]/2:, :, :]

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
