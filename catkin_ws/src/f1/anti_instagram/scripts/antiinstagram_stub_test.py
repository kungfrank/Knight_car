#!/usr/bin/env python

import numpy as np
from anti_instagram.AntiInstagram import *
import cv2
import timeit

from duckietown_utils.expand_variables import expand_environment

def compute_error(a, b):
	return np.mean(np.square(a * 1.0 - b * 1.0))

# calculate transform for each image set
def testImages(ai, imageset, gtimageset):
	error = []
	for i, image in enumerate(imageset):
		#print(image.shape)
		ai.calculateTransform(image,True)
		print('health: %s' % ai.health)
		transimage = ai.applyTransform(image)
		testimgf = "testimage%d.jpg" % i
		cv2.imwrite(testimgf,transimage)
		testimg = cv2.imread(testimgf)
		# uncorrected is > 500
		error = compute_error(testimg, gtimageset[i])
		error.append(error)
		if error[i] > 1:
			print("Correction seemed to fail for image # "+str(i))
	return error

def read_file(filename):
	filename = expand_environment(filename)
	img = cv2.imread(filename)
	if not img:
		msg = 'Cannot read filename %r.' % filename
		raise ValueError(msg)
	return img

def genRandImg():
	# determine size of image
	size = 480,640,3
	# add uniform noise
	img = np.array(np.random.random(size)*255, dtype= np.uint8)
	# return random image
	return img

def applyTransformOnRandomImg():
	t = timeit.timeit(stmt='ai.applyTransform(img)', 
		setup='import numpy as np; import anti_instagram.AntiInstagram as AntiInstagram; img = np.array(np.random.random((480,640,3))*255, dtype= np.uint8); ai = AntiInstagram.AntiInstagram()', 
		number=3
		)
	print("Averge Apply Transform Took: " + str(t/3.0 * 1000) + " milliseconds")

def calcuateTransformOnRandomImg():
	t = timeit.timeit(stmt='ai.calculateTransform(img,True)', 
		setup='import numpy as np; import anti_instagram.AntiInstagram as AntiInstagram; img = np.array(np.random.random((480,640,3))*255, dtype= np.uint8); ai = AntiInstagram.AntiInstagram()', 
		number=3
		)
	print("Average Calculate Transform Took: " + str(t/3.0 * 1000) + " milliseconds")





if __name__ == '__main__':
	ai = AntiInstagram()

	# set to non-zero if test failed
	exit_code = 0

	#### TEST SINGLE TRANSFORM ####
	# load test images
	imagesetf = [
	"inputimage0.jpg",
	"inputimage1.jpg"
	]
	gtimagesetf = [
	"groundtruthimage0.jpg",
	"groundtruthimage1.jpg"
	]
	#
	imageset = []
	for imgf in imagesetf:
		img = cv2.imread(imgf)
		imageset.append(img)
	gtimageset = []
	for imgf in gtimagesetf:
		img = cv2.imread(imgf)
		gtimageset.append(img)
	errors = testImages(ai,imageset,gtimageset)
	print("Test Image Errors (errors >> 1 indicate a problem): ")
	print(errors)

	if max(errors) > 3:
		exit_code = 1

	#### TEST CALC TIMING ####
	calcuateTransformOnRandomImg()
	applyTransformOnRandomImg()

	if exit_code != 0:
		raise Exception("Tests not passed!")
	print("All tests passed!")
