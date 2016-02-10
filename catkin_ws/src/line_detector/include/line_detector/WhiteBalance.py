import numpy as np
import cv2
import sys
import time
from matplotlib import pyplot as plt

class WhiteBalance(object):
    def __init__(self):
        self.x = 0.5
        self.y = 0.5
        self.w = 0.08
        self.h = 0.08
        self.norm_bgr = np.ones((1,1,3))         

    def __calculateRegion(self, shape):
        y1 = int((self.y - self.h/2)* shape[0])
        x1 = int((self.x - self.w/2)* shape[1])
        y2 = int((self.y + self.h/2)* shape[0])
        x2 = int((self.x + self.w/2)* shape[1])
        return x1, y1, x2, y2

    def setRefImg(self, bgr):
        # region that we assume to be gray
        x1, y1, x2, y2 = self.__calculateRegion(bgr.shape)
        region = np.copy(bgr[y1:y2, x1:x2, :])

        # Gray world assumption algorithm
        # mean or max
        self.norm_bgr = np.mean(region)/np.mean(region, axis=(0,1), keepdims=True)
        #self.norm_bgr = max(np.mean(region, axis=(0,1)))/np.mean(region, axis=(0,1), keepdims=True)
        print self.norm_bgr

    def correctImg(self, bgr):
        bgr *= self.norm_bgr
    
    def plotHist(self, bgr):
        color = ('b','g','r')
        for i,col in enumerate(color):
            histr = cv2.calcHist([bgr],[i],None,[256],[0,256])
            plt.plot(histr,color = col)
            plt.xlim([0,256])
        plt.show()

    def drawRegion(self, bgr):
        x1, y1, x2, y2 = self.__calculateRegion(bgr.shape)
        cv2.rectangle(bgr, (x1,y1), (x2,y2), (0,0,255)) 
 
def main():
    wb = WhiteBalance()

    if len(sys.argv)==2:
        bgr = cv2.imread(sys.argv[1])
        bgr = cv2.resize(bgr, (640, 480)) 
        
        # Set reference image to estimate the parameters
        wb.setRefImg(bgr)

        # Histogram before balancing
        wb.plotHist(bgr)
        
        # Apply white balancing
        wb.correctImg(bgr)
        wb.plotHist(bgr)

        wb.drawRegion(bgr)
        cv2.imshow('image', bgr)
        cv2.waitKey(0)

    else:
        print 'Please choose an image...'
        return -1
 
if __name__=='__main__':
    main() 
