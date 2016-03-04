import numpy as np
import cv2
import sys
import time

class LineDetector(object):
    def __init__(self):
        # Images to be processed
        self.bgr = np.empty(0)
        self.hsv = np.empty(0)
        self.edges = np.empty(0)

        # Color value range in HSV space: default
        self.hsv_white1 = np.array([0, 0, 150])
        self.hsv_white2 = np.array([180, 60, 255]) 
        self.hsv_yellow1 = np.array([25, 140, 100])
        self.hsv_yellow2 = np.array([45, 255, 255]) 
        self.hsv_red1 = np.array([0, 140, 100])
        self.hsv_red2 = np.array([15, 255, 255]) 
        self.hsv_red3 = np.array([165, 140, 100])
        self.hsv_red4 = np.array([180, 255, 255]) 

        # Parameters for dilation, Canny, and Hough transform: default
        self.dilation_kernel_size = 3
        self.canny_thresholds = [80,200]
        self.hough_threshold  = 20
        self.hough_min_line_length = 3
        self.hough_max_line_gap = 1

    def __colorFilter(self, color):
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

		# binary dilation
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE,(self.dilation_kernel_size, self.dilation_kernel_size))
        bw = cv2.dilate(bw, kernel)
        
        # refine edge for certain color
        edge_color = cv2.bitwise_and(bw, self.edges)

        return bw, edge_color

    def __findEdge(self, gray):
        edges = cv2.Canny(gray, self.canny_thresholds[0], self.canny_thresholds[1], apertureSize = 3)
        return edges

    def __HoughLine(self, edge):
        lines = cv2.HoughLinesP(edge, 1, np.pi/180, self.hough_threshold, np.empty(1), self.hough_min_line_length, self.hough_max_line_gap)
        if lines is not None:
            lines = np.array(lines[0])
        else:
            lines = []
        return lines
    
    def __checkBounds(self, val, bound):
        val[val<0]=0
        val[val>=bound]=bound-1
        return val

    def __correctPixelOrdering(self, lines, normals):
        flag = ((lines[:,2]-lines[:,0])*normals[:,1] - (lines[:,3]-lines[:,1])*normals[:,0])>0
        for i in range(len(lines)):
            if flag[i]:
                x1,y1,x2,y2 = lines[i, :]
                lines[i, :] = [x2,y2,x1,y1] 
 
    def __findNormal(self, bw, lines):
        normals = []
        if len(lines)>0:
            length = np.sum((lines[:, 0:2] -lines[:, 2:4])**2, axis=1, keepdims=True)**0.5
            dx = 1.* (lines[:,3:4]-lines[:,1:2])/length
            dy = 1.* (lines[:,0:1]-lines[:,2:3])/length
            x3 = ((lines[:,0:1]+lines[:,2:3])/2 - 3.*dx).astype('int')
            y3 = ((lines[:,1:2]+lines[:,3:4])/2 - 3.*dy).astype('int')
            x4 = ((lines[:,0:1]+lines[:,2:3])/2 + 3.*dx).astype('int')
            y4 = ((lines[:,1:2]+lines[:,3:4])/2 + 3.*dy).astype('int')
            x3 = self.__checkBounds(x3, bw.shape[1])
            y3 = self.__checkBounds(y3, bw.shape[0])
            x4 = self.__checkBounds(x4, bw.shape[1])
            y4 = self.__checkBounds(y4, bw.shape[0])
            flag_signs = (np.logical_and(bw[y3,x3]>0, bw[y4,x4]==0)).astype('int')*2-1
            normals = np.hstack([dx, dy]) * flag_signs
 
            """ # Old code with lists and loop, performs 4x slower 
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
            """
            self.__correctPixelOrdering(lines, normals)
        return normals

    def detectLines(self, color):
        bw, edge_color = self.__colorFilter(color)
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
                cv2.circle(self.bgr, (x1,y1), 2, (0,255,0))
                cv2.circle(self.bgr, (x2,y2), 2, (0,0,255))

    def drawNormals(self, lines, normals):
        if len(lines)>0:
            for x1,y1,x2,y2,dx,dy in np.hstack((lines,normals)):
                x3 = int((x1+x2)/2. - 4.*dx)
                y3 = int((y1+y2)/2. - 4.*dy)
                x4 = int((x1+x2)/2. + 4.*dx)
                y4 = int((y1+y2)/2. + 4.*dy)
                cv2.circle(self.bgr, (x3,y3), 3, (0,255,0))
                cv2.circle(self.bgr, (x4,y4), 3, (0,0,255))

    def getColorPixels(self, color):
        bw, edge_color = self.__colorFilter(color)
        return bw

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

            # show frame
            cv2.imshow('Line Detector', detector.getImage())
            cv2.imshow('Edge', detector.edges)
            cv2.waitKey(30)

    else:
        return -1

if __name__ == '__main__':
    _main()	
