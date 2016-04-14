import numpy as np
import cv2
import sys


class LineDetector2(object):
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

    def _colorFilter(self, color):
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
        #bw = cv2.dilate(bw, kernel)
        
        # refine edge for certain color
        edge_color = cv2.bitwise_and(cv2.dilate(bw, kernel), self.edges)

        return bw, edge_color

    def _lineFilter(self, bw, edge_color):
        # find gradient of the bw image
        grad_x = -cv2.Sobel(bw/255, cv2.CV_32F, 1, 0, ksize=5)
        grad_y = -cv2.Sobel(bw/255, cv2.CV_32F, 0, 1, ksize=5)
        grad_x *= (edge_color == 255)
        grad_y *= (edge_color == 255)

        grad = np.sqrt(grad_x**2 + grad_y**2)
        roi = (grad>40)

        #print np.unique(grad)
        #print np.sum(roi)

        # turn into a list of points and normals
        roi_y, roi_x = np.nonzero(roi)
        centers = np.vstack((roi_x, roi_y)).transpose()
        normals = np.vstack((grad_x[roi], grad_y[roi])).transpose()
        normals /= np.sqrt(np.sum(normals**2, axis=1, keepdims=True))

        #self.drawNormals(centers, normals, (0,0,0))
        lines = self._synthesizeLines(centers, normals)

        return lines, normals, centers

    def _findEdge(self, gray):
        edges = cv2.Canny(gray, self.canny_thresholds[0], self.canny_thresholds[1], apertureSize = 3)
        return edges

    def _HoughLine(self, edge):
        lines = cv2.HoughLinesP(edge, 1, np.pi/180, self.hough_threshold, np.empty(1), self.hough_min_line_length, self.hough_max_line_gap)
        if lines is not None:
            lines = np.array(lines[0])
        else:
            lines = []
        return lines
    
    def _checkBounds(self, val, bound):
        val[val<0]=0
        val[val>=bound]=bound-1
        return val

    def _correctPixelOrdering(self, lines, normals):
        flag = ((lines[:,2]-lines[:,0])*normals[:,1] - (lines[:,3]-lines[:,1])*normals[:,0])>0
        for i in range(len(lines)):
            if flag[i]:
                x1,y1,x2,y2 = lines[i, :]
                lines[i, :] = [x2,y2,x1,y1] 
 
    def _synthesizeLines(self, centers, normals):
        lines = []
        if len(centers)>0:
            x1 = (centers[:,0:1] + normals[:, 1:2] * 2.).astype('int')
            y1 = (centers[:,1:2] - normals[:, 0:1] * 2.).astype('int')
            x2 = (centers[:,0:1] - normals[:, 1:2] * 2.).astype('int')
            y2 = (centers[:,1:2] + normals[:, 0:1] * 2.).astype('int')
            x1 = self._checkBounds(x1, self.bgr.shape[1])
            y1 = self._checkBounds(y1, self.bgr.shape[0])
            x2 = self._checkBounds(x2, self.bgr.shape[1])
            y2 = self._checkBounds(y2, self.bgr.shape[0])
            lines = np.hstack([x1, y1, x2, y2])
        return lines

    def detectLines2(self, color):
        bw, edge_color = self._colorFilter(color)
        lines, normals, centers = self._lineFilter(bw, edge_color)
        return lines, normals, centers, bw

    def setImage(self, bgr):
        self.bgr = np.copy(bgr)
        self.hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
        self.edges = self._findEdge(self.bgr)
  
    def getImage(self):
        return self.bgr
 
    def drawLines(self, lines, paint):
        if len(lines)>0:
            for x1,y1,x2,y2 in lines:
                cv2.line(self.bgr, (x1,y1), (x2,y2), paint, 2)
                cv2.circle(self.bgr, (x1,y1), 2, (0,255,0))
                cv2.circle(self.bgr, (x2,y2), 2, (0,0,255))

    def drawNormals2(self, centers, normals, paint):
        if len(centers)>0:
            for x,y,dx,dy in np.hstack((centers,normals)):
                x3 = int(x - 2.*dx)
                y3 = int(y - 2.*dy)
                x4 = int(x + 2.*dx)
                y4 = int(y + 2.*dy)
                cv2.line(self.bgr, (x3,y3), (x4,y4), paint, 1)
                cv2.circle(self.bgr, (x3,y3), 1, (0,255,0))
                cv2.circle(self.bgr, (x4,y4), 1, (0,0,255))
#
# def _main():
#     detector = LineDetector2()
#     # read image from file or camera
#     if len(sys.argv)==2:
#         bgr = cv2.imread(sys.argv[1])
#
#         # crop and resize frame
#         #bgr = cv2.resize(bgr, (640, 480))
#         #bgr = bgr[160:, :, :]
#         bgr = cv2.resize(bgr, (160, 120), interpolation=cv2.INTER_CUBIC)
#         bgr = bgr[40:, :, :]
#
#         # set the image to be detected
#         detector.setImage(bgr)
#
#         # detect lines and normals
#         lines_white, normals_white, centers_white, area_white = detector.detectLines('white')
#         lines_yellow, normals_yellow, centers_yellow, area_yellow = detector.detectLines('yellow')
#         lines_red, normals_red, centers_red, area_red = detector.detectLines('red')
#
#         # draw lines
#         #detector.drawLines(lines_white, (0,0,0))
#         #detector.drawLines(lines_yellow, (255,0,0))
#         #detector.drawLines(lines_red, (0,255,0))
#
#         # draw normals
#         detector.drawNormals(centers_white, normals_white, (0,0,0))
#         detector.drawNormals(centers_yellow, normals_yellow, (255,0,0))
#         detector.drawNormals(centers_red, normals_red, (0,255,0))
#
#         #cv2.imwrite('lines_with_normal.png', detector.getImage())
#         cv2.imshow('frame', cv2.resize(detector.getImage(), (640,320), interpolation=cv2.INTER_CUBIC))
#         #cv2.imshow('edge', detector.edges)
#         #cv2.imshow('area_white', area_white)
#
#         cv2.waitKey(0)
#
#     elif len(sys.argv)==1:
#         cap = cv2.VideoCapture(0)
#         if not cap.isOpened():
#             print 'Error opening camera...'
#             return -1
#
#         while True:
#             ret, bgr = cap.read()
#             if not ret:
#                 print 'No frames grabbed...'
#                 break
#
#             # crop and resize frame
#             bgr = cv2.resize(bgr, (160, 120))
#
#             # set the image to be detected
#             detector.setImage(bgr)
#
#             # detect lines and normals
#             lines_white, normals_white, centers_white, area_white = detector.detectLines('white')
#             lines_yellow, normals_yellow, centers_yellow, area_yellow = detector.detectLines('yellow')
#             lines_red, normals_red, centers_red, area_red = detector.detectLines('red')
#
#             # draw lines
#             #detector.drawLines(lines_white, (0,0,0))
#             #detector.drawLines(lines_yellow, (255,0,0))
#             #detector.drawLines(lines_red, (0,255,0))
#
#             # draw normals
#             detector.drawNormals(centers_white, normals_white, (0,0,0))
#             detector.drawNormals(centers_yellow, normals_yellow, (255,0,0))
#             detector.drawNormals(centers_red, normals_red, (0,255,0))
#
#             # show frame
#             cv2.imshow('Line Detector', detector.getImage())
#             cv2.imshow('Edge', detector.edges)
#             cv2.waitKey(30)
#
#     else:
#         return -1
#
# if __name__ == '__main__':
#     _main()
