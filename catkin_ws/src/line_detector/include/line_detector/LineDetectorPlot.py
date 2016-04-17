import numpy as np
import cv2
import sys

# draw line segments
def drawLines(bgr, lines, paint):
    if len(lines)>0:
        for x1,y1,x2,y2 in lines:
            cv2.line(bgr, (x1,y1), (x2,y2), paint, 2)
            cv2.circle(bgr, (x1,y1), 2, (0,255,0))
            cv2.circle(bgr, (x2,y2), 2, (0,0,255))

# draw segment normals
def drawNormals(bgr, lines, normals):
    if len(lines)>0:
        for x1,y1,x2,y2,dx,dy in np.hstack((lines,normals)):
            x3 = int((x1+x2)/2. - 4.*dx)
            y3 = int((y1+y2)/2. - 4.*dy)
            x4 = int((x1+x2)/2. + 4.*dx)
            y4 = int((y1+y2)/2. + 4.*dy)
            cv2.circle(bgr, (x3,y3), 2, (0,255,0))
            cv2.circle(bgr, (x4,y4), 2, (0,0,255))

# draw segment normals2
def drawNormals2(bgr, centers, normals, paint):
    if len(centers)>0:
        for x,y,dx,dy in np.hstack((centers,normals)):
            x3 = int(x - 2.*dx)
            y3 = int(y - 2.*dy)
            x4 = int(x + 2.*dx)
            y4 = int(y + 2.*dy)
            cv2.line(bgr, (x3,y3), (x4,y4), paint, 1)
            cv2.circle(bgr, (x3,y3), 1, (0,255,0))
            cv2.circle(bgr, (x4,y4), 1, (0,0,255))
