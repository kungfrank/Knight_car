import numpy as np
import cv2
import sys
import time

class LineDetector(object):
    def __init__(self):
        self.hsv_white1 = np.array([0, 0, 220])
        self.hsv_white2 = np.array([255, 25, 255]) 
        self.hsv_yellow1 = np.array([25, 100, 200])
        self.hsv_yellow2 = np.array([45, 255, 255]) 
        self.hsv_red1 = np.array([0, 100, 120])
        self.hsv_red2 = np.array([10, 255, 255]) 
        self.hsv_red3 = np.array([245, 100, 120])
        self.hsv_red4 = np.array([255, 255, 255]) 

    def __colorFilter(self, bgr, color):
        # tic = time.time()
        # filter colors in HSV space
        hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
        if color == 'white':
            lane = cv2.inRange(hsv, self.hsv_white1, self.hsv_white2)
        elif color == 'yellow':
            lane = cv2.inRange(hsv, self.hsv_yellow1, self.hsv_yellow2)
        elif color == 'red':
            lane1 = cv2.inRange(hsv, self.hsv_red1, self.hsv_red2)
            lane2 = cv2.inRange(hsv, self.hsv_red3, self.hsv_red4)
            lane = cv2.bitwise_or(lane1, lane2)
        else:
	        raise Exception('Error: Undefined color strings...')
        # print 'Color thresholding:' + str(time.time()-tic)

		# binary image processing
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE,(2, 2))
        lane = cv2.erode(lane, kernel)
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE,(4, 4))
        lane = cv2.dilate(lane, kernel)

        return lane

    def __findEdge(self, gray):	
        edges = cv2.Canny(gray, 10, 30, apertureSize = 3)
        return edges

    def __HoughLine(self, edge, bgr):
        lines = cv2.HoughLinesP(edge, 1, np.pi/180, 30, np.empty(1), minLineLength=10, maxLineGap=1)
        if lines is not None:
            lines = lines[0]
        else:
            lines = []
        return lines
    
    def __checkBounds(self, val, bound):
        if val<0:
            val = 0
        elif val>=bound:
            val = bound-1
        return val

    def __correctPixelOrdering(self, lines, normals):
       for i in range(len(lines)):
            x1,y1,x2,y2 = lines[i, :]
            dx, dy = normals[i, :]
            if (x2-x1)*dy-(y2-y1)*dx<0:
                lines[i, :] = [x2,y2,x1,y1]

    def __findNormal(self, bw, lines):
        normals = []
        if len(lines)>0:
            normals = np.zeros((len(lines), 2))
            for cnt,line in enumerate(lines):
                x1,y1,x2,y2 = line
                dx = 1.*(y2-y1)/((x1-x2)**2+(y1-y2)**2)**0.5
                dy = 1.*(x1-x2)/((x1-x2)**2+(y1-y2)**2)**0.5
                x3 = int((x1+x2)/2. - 2.*dx)
                y3 = int((y1+y2)/2. - 2.*dy)
                x4 = int((x1+x2)/2. + 2.*dx)
                y4 = int((y1+y2)/2. + 2.*dy)
                x3 = self.__checkBounds(x3, bw.shape[1])
                y3 = self.__checkBounds(y3, bw.shape[0])
                x4 = self.__checkBounds(x4, bw.shape[1])
                y4 = self.__checkBounds(y4, bw.shape[0])
                if bw[y3,x3]>0 and bw[y4,x4]==0:
                    normals[cnt,:] = [dx, dy] 
                else:
                    normals[cnt,:] = [-dx, -dy]

            self.__correctPixelOrdering(lines, normals)
        return normals

    def detectLines(self, bgr, color):
        bw = self.__colorFilter(bgr, color)
        edges = self.__findEdge(bw)
        lines = self.__HoughLine(edges, bgr)
        normals = self.__findNormal(bw, lines)
        return lines, normals

    def drawLines(self, bgr, lines, paint):
        if len(lines)>0:
            for x1,y1,x2,y2 in lines:
                cv2.line(bgr, (x1,y1), (x2,y2), paint, 2)

    def drawNormals(self, bgr, lines, normals):
        if len(lines)>0:
            for x1,y1,x2,y2,dx,dy in np.hstack((lines,normals)):
                x3 = int((x1+x2)/2. - 4.*dx)
                y3 = int((y1+y2)/2. - 4.*dy)
                x4 = int((x1+x2)/2. + 4.*dx)
                y4 = int((y1+y2)/2. + 4.*dy)
                x3 = self.__checkBounds(x3, bgr.shape[1])
                y3 = self.__checkBounds(y3, bgr.shape[0])
                x4 = self.__checkBounds(x4, bgr.shape[1])
                y4 = self.__checkBounds(y4, bgr.shape[0])
                cv2.circle(bgr, (x3,y3), 3, (0,255,0))
                cv2.circle(bgr, (x4,y4), 3, (0,0,255))

    def getLane(self, bgr, color):
        bw = self.__colorFilter(bgr, color)
        return bw

def _main():
    # read image from file or camera
    if len(sys.argv)==2:
        bgr = cv2.imread(sys.argv[1])
        
        # crop and resize frame
        bgr = cv2.resize(bgr, (200, 150))
        bgr = bgr[bgr.shape[0]/2:, :, :]

        # detect lines and normals
        detector = LineDetector()
        lines_white, normals_white = detector.detectLines(bgr, 'white')
        lines_yellow, normals_yellow = detector.detectLines(bgr, 'yellow')
        lines_red, normals_red = detector.detectLines(bgr, 'red')

        # get lanes
        # bw = detector.getLane(bgr, 'white')
        # cv2.imshow('white lane', bw)

        # draw lines
        detector.drawLines(bgr, lines_white, (0,0,0))
        detector.drawLines(bgr, lines_yellow, (255,0,0))
        detector.drawLines(bgr, lines_red, (0,255,0))
       
        # draw normals
        detector.drawNormals(bgr, lines_yellow, normals_yellow)
        detector.drawNormals(bgr, lines_white, normals_white)
        detector.drawNormals(bgr, lines_red, normals_red)
        
        cv2.imwrite('lines_with_normal.png', bgr) 
        cv2.imshow('frame', bgr)
        cv2.waitKey(0)

    elif len(sys.argv)==1:
        cap = cv2.VideoCapture(0)
        if not cap.isOpened():
            print 'Error opening camera...'
            return -1

        while True:
            ret, bgr = cap.read()
            if not ret:
                print 'No frames grabbed...'
                break

            # crop and resize frame
            bgr = cv2.resize(bgr, (200, 150))
            #bgr = bgr[bgr.shape[0]/2:, :, :]
            
            # detect lines and normals
            detector = LineDetector()
            lines_white, normals_white = detector.detectLines(bgr, 'white')
            lines_yellow, normals_yellow = detector.detectLines(bgr, 'yellow')
            lines_red, normals_red = detector.detectLines(bgr, 'red')
            
            # draw lines
            detector.drawLines(bgr, lines_white, (0,0,0))
            detector.drawLines(bgr, lines_yellow, (255,0,0))
            detector.drawLines(bgr, lines_red, (0,255,0))
           
            # draw normals
            detector.drawNormals(bgr, lines_yellow, normals_yellow)
            detector.drawNormals(bgr, lines_white, normals_white)
            detector.drawNormals(bgr, lines_red, normals_red)

            cv2.imshow('Line Detector', bgr)
            cv2.waitKey(30)

    else:
        return -1

if __name__ == '__main__':
    _main()	
