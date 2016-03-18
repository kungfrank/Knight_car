import kmeans

class AntiInstagram():
	def __init__(self):
		self.num_colors = 3
		self.ideal_colors = [[0, 0, 0], [0, 255, 255], [255, 255, 255]]
		self.transformation = [0,0,0]
		self.scale = [0,0,0]
		self.shift = [0,0,0]
		self.health = 0



	def applyTransformation(self,image):
		corrected_image = scaleandshift(img,self.scale,self.shift)
		return corrected_image

	def calculateTransform(self,image):
		trained,counter = kmeans.runKMeans(image)
		mapping = kmeans.identifyColors(trained, kmeans.CENTERS)
		r,g,b = kmeans.getparameters(mapping, trained, kmeans.CENTERS)

		self.scale = [r[0][0][0],g[0][0][0],b[0][0][0]]
		self.shift = [r[1][0], g[1][0],b[1][0]]

		return 1

	def calculateHealth(self):
		return self.health
		