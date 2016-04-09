#!/usr/bin/env python
from anti_instagram.AntiInstagram import *
import numpy as np
import cv2
from duckietown_utils.expand_variables import expand_environment


# calculate transform for each image set
def testImages(ai, imageset, gtimageset):
	for i,image in enumerate(imageset):
		ai.calculateTransform(image,True)
		print ai.health
		transimage = ai.applyTransform(image)
		cv2.imwrite("testimage"+str(i)+".jpg",transimage)
		# uncorrected is > 500
		error = np.mean(np.square(transimage-gtimageset[i]))
		if error > 500:
			print("Correction seemed to fail for image # "+str(i))


def read_file(filename):
	filename = expand_environment(filename)
	img = cv2.imread(filename)
	if not img:
		msg = 'Cannot read filename %r.' % filename
		raise ValueError(msg)
	return img

if __name__ == '__main__':
	ai = AntiInstagram()

	# load test images
	imagesetf = [
		"${DUCKIETOWN_DATA}/phase_2/f1-illum-robust/tristan_manual_dataset1/tristan/frame0001.jpg",
		"${DUCKIETOWN_DATA}/phase_2/f1-illum-robust/tristan_manual_dataset1/lapentab/frame0002.jpg"
	]

	gtimagesetf = [
		"groundtruthimage0.jpg",
		"groundtruthimage1.jpg"
	]
	
	imageset = filter(read_file, imagesetf)
	gtimageset = filter(read_file, gtimagesetf)

	testImages(ai, imageset, gtimageset)
