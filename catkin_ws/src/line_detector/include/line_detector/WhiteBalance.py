import numpy as np
import cv2
import sys
import time

class WhiteBalance(object):
    def __init__(self):
        self.x = 0.5
        self.y = 0.7
        self.w = 0.16
        self.h = 0.16
        self.norm_bgr = np.ones((1,1,3))         

    def __calculateRegion(self, shape):
        y1 = int((self.y - self.h/2)* shape[0])
        x1 = int((self.x - self.w/2)* shape[1])
        y2 = int((self.y + self.h/2)* shape[0])
        x2 = int((self.x + self.w/2)* shape[1])
        return x1, y1, x2, y2

    def setRefImg(self, bgr):
        x1, y1, x2, y2 = self.__calculateRegion(bgr.shape)

        # region that we assume to be gray
        region = np.copy(bgr[y1:y2, x1:x2, :])
        self.norm_bgr = np.mean(region, axis=(0,1), keepdims=True)/np.mean(region)
        print self.norm_bgr

    def correctImg(self, bgr):
        bgr = (1.*bgr/self.norm_bgr).astype('uint8')

    def drawRegion(self, bgr):
        x1, y1, x2, y2 = self.__calculateRegion(bgr.shape)
        cv2.rectangle(bgr, (x1,y1), (x2,y2), (0,0,255)) 
 
def main():
    wb = WhiteBalance()

    if len(sys.argv)==2:
        bgr = cv2.imread(sys.argv[1])
        bgr = cv2.resize(bgr, (320, 240)) 
        wb.setRefImg(bgr)

        wb.correctImg(bgr)

        wb.drawRegion(bgr)
        cv2.imshow('image', bgr)
        cv2.waitKey(0)

    else:
        print 'Please choose an image...'
        return -1
 
if __name__=='__main__':
    main() 
