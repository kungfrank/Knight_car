

class AntiInstagram(object):
	def __init__(self):
		self.num_colors = 3
		self.ideal_colors = [[0, 0, 0], [0, 255, 255], [255, 255, 255]]
		self.transformation = [0,0,0]
		self.health = 0


	def applyTransformation(self,image):





		return corrected_image

	def calculateHealth(self):
		'''one way is to keep one img every 1 sec and compare the diff http://stackoverflow.com/questions/189943/how-can-i-quantify-difference-between-two-images'''

		

		return self.health
		

	def calibrateLighting(self, cal_img):
		sample = self.getSample(cal_img)

		equalParam = self.histEqual(sample)

		kMeansParam = self.kMeansParam(sample)

		return self.calibration_parameters

	def getSample(self, img):
		'''code to sample from entire img a part that contains lanes and floor'''

		return sample 

	def histEqual(self, img):
		'''https://en.wikipedia.org/wiki/Normalization_(image_processing)'''

		return equalParam

	def kMeans(self, img):

		return kMeansParam
