#!/usr/bin/env python

import numpy as np
from anti_instagram.AntiInstagram import *
import cv2
import timeit

from duckietown_utils.expand_variables import expand_environment

# calculate transform for each image set
def testImages(ai, imageset, gtimageset):
	error = []
	for i,image in enumerate(imageset):
		#print(image.shape)
		ai.calculateTransform(image,True)
		print ai.health
		transimage = ai.applyTransform(image)
		testimgf = "testimage"+str(i)+".jpg"
		cv2.imwrite(testimgf,transimage)
		testimg = cv2.imread(testimgf)
		# uncorrected is > 500
		error.append(np.mean(np.square(testimg-gtimageset[i])))
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
	print("Test Image Errors (errors > 1 indicate a problem): ")
	print(errors)

	#### TEST CALC TIMING ####
	calcuateTransformOnRandomImg()
	applyTransformOnRandomImg()