from __future__ import print_function

import numpy as np
import cv2
import kmeans

# Load an color image in grayscale
#cref = [50.0,50.0,50.0]
def refPatchScale(image_fname, cref):
	img_orig = cv2.imread(image_fname)
	h = img_orig.shape[0]
	w = img_orig.shape[1]

	c1 = 250
	r1 = 320
	img_patch = img_orig[c1:c1+50,r1:r1+100,:]
	hp = img_patch.shape[0]
	wp = img_patch.shape[1]

	# just for testing
	cv2.imwrite("img_patch.jpg",img_patch)

	m_mean = np.zeros(3)
	m_mean = np.mean(np.reshape(img_patch,[wp*hp,3]),0)

	print(m_mean)

	mscale = np.array(cref) / np.array(m_mean)

	print(mscale)

	# reshape to do fast multiply
	img_scale = np.reshape(img_orig,[h*w,3])
	img_scale = np.reshape(img_scale*mscale,[h,w,3])

	return img_scale

def scaleandshift(img,scale,shift):
	h = img.shape[0]
	w = img.shape[1]

	img_scale = np.reshape(img,[h*w,3])
	img_scale = np.reshape(img_scale*np.array(scale),[h,w,3])

	img_shift = np.reshape(img_scale,[h*w,3])
	img_shift = np.reshape(img_shift+np.array(shift),[h,w,3])

	return img_shift

def refPatchShift(image_fname,cref):
	img_orig = cv2.imread(image_fname)
	h = img_orig.shape[0]
	w = img_orig.shape[1]

	c1 = 300
	r1 = 300
	img_patch = img_orig[c1:c1+100,r1:r1+100,:]
	hp = img_patch.shape[0]
	wp = img_patch.shape[1]

	m_mean = np.zeros(3)
	m_mean = np.mean(np.reshape(img_patch,[wp*hp,3]),0)

	print(m_mean)

	mshift = np.array(cref) - np.array(m_mean)

	print(mshift)

	img_shift = np.reshape(img_orig,[h*w,3])
	img_shift = np.reshape(img_shift+mshift,[h,w,3])

	return img_shift


# cv2.imwrite("fname_out.jpg",img_scale_n_shift)