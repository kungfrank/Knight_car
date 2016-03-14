import numpy as np
import cv2

import sys

# Load an color image in grayscale
c1 = 420
r1 = 230
cref = [50.0,50.0,50.0]

img_orig = cv2.imread(sys.argv[1])
img = img_orig[c1:c1+25,r1:r1+25,:]

#m_mean = cref

# can do in one line with np?
m_mean = np.zeros(3)
m_mean[0] = cv2.mean(img[:,:,0])[0]
m_mean[1] = cv2.mean(img[:,:,1])[0]
m_mean[2] = cv2.mean(img[:,:,2])[0]

print(m_mean)

mscale = np.array(cref) / np.array(m_mean)

print(mscale)

# reshape to do cleaner multiply
img_scale = img_orig
img_scale[:,:,0] = img_orig[:,:,0]*mscale[0]
img_scale[:,:,1] = img_orig[:,:,1]*mscale[1]
img_scale[:,:,2] = img_orig[:,:,2]*mscale[2]

cv2.imwrite(sys.argv[2],img_scale)