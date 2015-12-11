from picamera.array import PiRGBArray
from picamera import PiCamera
import cv2
import laneDetector

if __name__ == '__main__':
 
	resolution = (640, 480)
	fps = 30

	# initialize the camera
	camera = PiCamera()
	camera.resolution = resolution
	camera.framerate = fps
	rawCapture = PiRGBArray(camera, size=resolution)
 
	# capture frames from the camera
	for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
		bgr = frame.array
		
		# Line detector
		lines = laneDetector.lineDetector(bgr)
 
		# show the frame
		cv2.imshow("Frame", bgr)
		cv2.waitKey(1)
 
		# clear the stream in preparation for the next frame
		rawCapture.truncate(0)
