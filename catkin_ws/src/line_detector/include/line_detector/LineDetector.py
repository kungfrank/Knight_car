import numpy as np
import cv2
import sys
import time

class LineDetector(object):
    def __init__(self):
        self.bgr = []
        self.hsv = []

        # self.hei = 200
        # self.wid = 320
        # self.top_cutoff = 80

        # Color value range in HSV space
        self.hsv_white1 = np.array([0, 0, 200])
        self.hsv_white2 = np.array([255, 100, 255]) 
        self.hsv_yellow1 = np.array([25, 150, 150])
        self.hsv_yellow2 = np.array([45, 255, 255]) 
        self.hsv_red1 = np.array([0, 100, 120])
        self.hsv_red2 = np.array([10, 255, 255]) 
        self.hsv_red3 = np.array([245, 100, 120])
        self.hsv_red4 = np.array([255, 255, 255]) 

    def __colorFilter(self, color):
        # tic = time.time()
        # threshold colors in HSV space
        if color == 'white':
            bw = cv2.inRange(self.hsv, self.hsv_white1, self.hsv_white2)
        elif color == 'yellow':
            bw = cv2.inRange(self.hsv, self.hsv_yellow1, self.hsv_yellow2)
        elif color == 'red':
            bw1 = cv2.inRange(self.hsv, self.hsv_red1, self.hsv_red2)
            bw2 = cv2.inRange(self.hsv, self.hsv_red3, self.hsv_red4)
            bw = cv2.bitwise_or(bw1, bw2)
        else:
	        raise Exception('Error: Undefined color strings...')
        # print 'Color thresholding:' + str(time.time()-tic)

		# binary dilation
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE,(3, 3))
        bw = cv2.dilate(bw, kernel)
        
        # refine edge
        edge_color = cv2.bitwise_and(bw, self.edges)

        return bw, edge_color

    def __findEdge(self, gray):	
        edges = cv2.Canny(gray, 80, 200, apertureSize = 3)
        return edges

    def __HoughLine(self, edge):
        lines = cv2.HoughLinesP(edge, 1, np.pi/180, 20, np.empty(1), minLineLength=3, maxLineGap=1)
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
            if (x2-x1)*dy-(y2-y1)*dx>0:
                lines[i, :] = [x2,y2,x1,y1]

    def __findNormal(self, bw, lines):
        normals = []
        if len(lines)>0:
            normals = np.zeros((len(lines), 2))
            for cnt,line in enumerate(lines):
                x1,y1,x2,y2 = line
                dx = 1.*(y2-y1)/((x1-x2)**2+(y1-y2)**2)**0.5
                dy = 1.*(x1-x2)/((x1-x2)**2+(y1-y2)**2)**0.5
                x3 = int((x1+x2)/2. - 3.*dx)
                y3 = int((y1+y2)/2. - 3.*dy)
                x4 = int((x1+x2)/2. + 3.*dx)
                y4 = int((y1+y2)/2. + 3.*dy)
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

    def detectLines(self, color):
        bw, edge_color = self.__colorFilter(color)
        # edges = self.__findEdge(bw)
        lines = self.__HoughLine(edge_color)
        normals = self.__findNormal(bw, lines)
        return lines, normals

    def setImage(self, bgr):
        self.bgr = np.copy(bgr)
        self.hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
        self.edges = self.__findEdge(self.bgr)
  
    def getImage(self):
        return self.bgr
 
    def drawLines(self, lines, paint):
        if len(lines)>0:
            for x1,y1,x2,y2 in lines:
                cv2.line(self.bgr, (x1,y1), (x2,y2), paint, 2)
                cv2.circle(self.bgr, (x1,y1), 3, (0,255,0))
                cv2.circle(self.bgr, (x2,y2), 3, (0,0,255))

    def drawNormals(self, lines, normals):
        if len(lines)>0:
            for x1,y1,x2,y2,dx,dy in np.hstack((lines,normals)):
                x3 = int((x1+x2)/2. - 4.*dx)
                y3 = int((y1+y2)/2. - 4.*dy)
                x4 = int((x1+x2)/2. + 4.*dx)
                y4 = int((y1+y2)/2. + 4.*dy)
                x3 = self.__checkBounds(x3, self.bgr.shape[1])
                y3 = self.__checkBounds(y3, self.bgr.shape[0])
                x4 = self.__checkBounds(x4, self.bgr.shape[1])
                y4 = self.__checkBounds(y4, self.bgr.shape[0])
                #cv2.circle(self.bgr, (x3,y3), 3, (0,255,0))
                #cv2.circle(self.bgr, (x4,y4), 3, (0,0,255))

    def getLane(self, color):
        return self.__colorFilter(self.bgr, color)

def _main():
    detector = LineDetector()
    # read image from file or camera
    if len(sys.argv)==2:
        bgr = cv2.imread(sys.argv[1])
        
        # crop and resize frame
        bgr = cv2.resize(bgr, (200, 150))
        bgr = bgr[bgr.shape[0]/2:, :, :]

        # set the image to be detected 
        detector.setImage(bgr)

        # detect lines and normals
        lines_white, normals_white = detector.detectLines('white')
        lines_yellow, normals_yellow = detector.detectLines('yellow')
        lines_red, normals_red = detector.detectLines('red')
        
        # draw lines
        detector.drawLines(lines_white, (0,0,0))
        detector.drawLines(lines_yellow, (255,0,0))
        detector.drawLines(lines_red, (0,255,0))
       
        # draw normals
        detector.drawNormals(lines_yellow, normals_yellow)
        detector.drawNormals(lines_white, normals_white)
        detector.drawNormals(lines_red, normals_red)

        cv2.imwrite('lines_with_normal.png', detector.getImage()) 
        cv2.imshow('frame', detector.getImage())
        cv2.imshow('edge', detector.edges)
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
          
            # set the image to be detected 
            detector.setImage(bgr)
 
            # detect lines and normals
            lines_white, normals_white = detector.detectLines('white')
            lines_yellow, normals_yellow = detector.detectLines('yellow')
            lines_red, normals_red = detector.detectLines('red')
            
            # draw lines
            detector.drawLines(lines_white, (0,0,0))
            detector.drawLines(lines_yellow, (255,0,0))
            detector.drawLines(lines_red, (0,255,0))
           
            # draw normals
            detector.drawNormals(lines_yellow, normals_yellow)
            detector.drawNormals(lines_white, normals_white)
            detector.drawNormals(lines_red, normals_red)

            cv2.imshow('Line Detector', detector.getImage())
            cv2.imshow('Edge', detector.edges)
            cv2.waitKey(30)

    else:
        return -1

if __name__ == '__main__':
    _main()	
