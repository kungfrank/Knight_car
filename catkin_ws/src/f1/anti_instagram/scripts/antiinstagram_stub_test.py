import numpy as np
from anti_instagram.AntiInstagram import *
import cv2
import timeit

from duckietown_utils.expand_variables import expand_environment

# calculate transform for each image set
def testImages(ai, imageset, gtimageset):
	for i,image in enumerate(imageset):
		#print(image.shape)
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
		number=10
		)
	print("Averge Apply Transform Took: " + str(t/10.0 * 1000) + " milliseconds")

def calcuateTransformOnRandomImg():
	t = timeit.timeit(stmt='ai.calculateTransform(img,True)', 
		setup='import numpy as np; import anti_instagram.AntiInstagram as AntiInstagram; img = np.array(np.random.random((480,640,3))*255, dtype= np.uint8); ai = AntiInstagram.AntiInstagram()', 
		number=10
		)
	print("Average Calculate Transform Took: " + str(t/10.0 * 1000) + " milliseconds")





if __name__ == '__main__':
	#ai = AntiInstagram()

	#### TEST CALC TIMING ####
	calcuateTransformOnRandomImg()
	applyTransformOnRandomImg()

	#### TEST SINGLE TRANSFORM ####
	# For future, incorporate with yaml
	# load test images
	#imagesetf = [
	#"/home/tristan/Dropbox/duckietown-data/phase_2/f1-illum-robust/tristan_manual_dataset1/tristan/frame0001.jpg",
	#"/home/tristan/Dropbox/duckietown-data/phase_2/f1-illum-robust/tristan_manual_dataset1/lapentab/frame0002.jpg"
	#]
	#gtimagesetf = [
	#"groundtruthimage0.jpg",
	#"groundtruthimage1.jpg"
	#]
	#
	#imageset = []
	#for imgf in imagesetf:
	#	img = cv2.imread(imgf)
	#	imageset.append(img)
	#gtimageset = []
	#for imgf in gtimagesetf:
	#	img = cv2.imread(imgf)
	#	gtimageset.append(img)
	#testImages(ai,gtimageset,gtimageset)