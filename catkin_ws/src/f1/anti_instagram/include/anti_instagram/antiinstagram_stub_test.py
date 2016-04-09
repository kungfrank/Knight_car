from AntiInstagram import *
import numpy as np
import cv2


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



if __name__ == '__main__':
	ai = AntiInstagram()

	# load test images
	imagesetf = [
	"/home/tristan/Dropbox/duckietown-data/phase_2/f1-illum-robust/tristan_manual_dataset1/tristan/frame0001.jpg",
	"/home/tristan/Dropbox/duckietown-data/phase_2/f1-illum-robust/tristan_manual_dataset1/lapentab/frame0002.jpg"
	]

	gtimagesetf = [
	"groundtruthimage0.jpg",
	"groundtruthimage1.jpg"
	]
	

	imageset = []
	for imgf in imagesetf:
		img = cv2.imread(imgf)
		imageset.append(img)

	gtimageset = []
	for imgf in gtimagesetf:
		img = cv2.imread(imgf)
		gtimageset.append(img)

	testImages(ai,imageset,gtimageset)