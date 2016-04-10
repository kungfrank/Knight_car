#!/usr/bin/env python
from anti_instagram import (AntiInstagram, L2_image_distance, logger,
	random_image, wrap_test_main)
from anti_instagram.utils import load_image
from duckietown_utils import col_logging  # @UnusedImport
import cv2
import sys
import timeit
import traceback



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
		e = L2_image_distance(testimg, gtimageset[i])
		if e > 1:
			logger.error("Correction seemed to fail for image %s" % i)
		error.append(e)
	return error

def setup():
	img = random_image(480, 640)
	ai = AntiInstagram()
	return ai, img

def applyTransformOnRandomImg():
	n = 3
	t = timeit.timeit(stmt='ai.applyTransform(img)', 
		setup='from __main__ import setup; ai,img=setup()', 
		number=n
		)
	logger.info("Average Apply Transform Took: %.1f ms "  % (t / n * 1000))
	
def calcuateTransformOnRandomImg():
	n = 3
	t = timeit.timeit(stmt='ai.calculateTransform(img,True)', 
					setup='from __main__ import setup; ai,img=setup()',
					number=n)
	logger.info("Average Calculate Transform Took: %.1f ms" % (t / n * 1000))

def anti_instagram_test():
	ai = AntiInstagram()
	failure = False
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

	if max(errors) > 3:
		failure = True

	#### TEST CALC TIMING ####
	calcuateTransformOnRandomImg()
	applyTransformOnRandomImg()

	if failure:
		logger.error("Tests not passed!")
		raise Exception("Tests not passed!")

	logger.info("All tests passed!")
	
if __name__ == '__main__':
	wrap_test_main(anti_instagram_test)
