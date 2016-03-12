#import random
#def getName():
#	return "Nick"
#def getStatus():
#	return random.choice(["happy","awesome"])

import numpy as np
import cv2
import sys
import time

def _main():
    # read image from file or camera
        bgr = cv2.imread(sys.argv[1])
	bgrMirror = cv2.flip(bgr)
        
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
        cv2.waitKey(0)


if __name__ == '__main__':
    _main()	
