#!/usr/bin/env python

from anti_instagram import logger
import traceback, sys
from anti_instagram.AntiInstagram import *
from duckietown_utils import col_logging  # @UnusedImport
from duckietown_utils.expand_variables import expand_environment
import cv2
import timeit


def compute_error(a, b):
	return np.mean(np.square(a * 1.0 - b * 1.0))

# calculate transform for each image set
def testImages(ai, imageset, gtimageset):
	error = []
	for i, image in enumerate(imageset):
		#print(image.shape)
		ai.calculateTransform(image,True)
		logger.info('health: %s' % ai.health)
		transimage = ai.applyTransform(image)
		testimgf = "testimage%d.jpg" % i
		cv2.imwrite(testimgf,transimage)
		testimg = cv2.imread(testimgf)
		# uncorrected is > 500
		e = compute_error(testimg, gtimageset[i])
		if e > 1:
			logger.error("Correction seemed to fail for image %s" % i)
		error.append(e)
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
	size = 480, 640, 3
	# add uniform noise
	img = np.array(np.random.random(size)*255, dtype= np.uint8)
	# return random image
	return img

def applyTransformOnRandomImg():
	t = timeit.timeit(stmt='ai.applyTransform(img)', 
		setup='import numpy as np; import anti_instagram.AntiInstagram as AntiInstagram; img = np.array(np.random.random((480,640,3))*255, dtype= np.uint8); ai = AntiInstagram.AntiInstagram()', 
		number=3
		)
	logger.info("Averge Apply Transform Took: " + str(t / 3.0 * 1000) + " milliseconds")

def calcuateTransformOnRandomImg():
	t = timeit.timeit(stmt='ai.calculateTransform(img,True)', 
		setup='import numpy as np; import anti_instagram.AntiInstagram as AntiInstagram; img = np.array(np.random.random((480,640,3))*255, dtype= np.uint8); ai = AntiInstagram.AntiInstagram()', 
		number=3
		)
	logger.info("Average Calculate Transform Took: " + str(t / 3.0 * 1000) + " milliseconds")


def load_image(f):
	""" Loads an image using cv2.imread and raise exception if not found """
	img = cv2.imread(f)
	if  img is None:
		msg = 'Could not read %r' % f
		raise ValueError(msg)
	return img

def anti_instagram_test():
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
	imageset = map(load_image, imagesetf)
	gtimageset = map(load_image, gtimagesetf)
	errors = testImages(ai, imageset, gtimageset)
	logger.info("Test Image Errors (errors >> 1 indicate a problem): ")
	logger.info(errors)

	if max(errors) > 500:
		exit_code = 1

	#### TEST CALC TIMING ####
	calcuateTransformOnRandomImg()
	applyTransformOnRandomImg()

	if exit_code != 0:
		logger.error("Tests not passed!")
		raise Exception("Tests not passed!")

	logger.info("All tests passed!")
	
if __name__ == '__main__':
	try:
		anti_instagram_test()
	except Exception as e:
		logger.error(traceback.format_exc(e))
		logger.error('Exiting with error code 1')
		sys.exit(1)
	except: # another weird exception
		logger.error('Exiting with error code 2')
		sys.exit(2)
	else:
		logger.info('Success.')
		sys.exit(0)
		
