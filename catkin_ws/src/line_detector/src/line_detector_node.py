#!/usr/bin/env python
import rospy
import cv2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from LineDetector import *
# from picamera.array import PiRGBArray
# from picamera import PiCamera

class LineDetectorNode(object):
    def __init__(self):
        self.bridge = CvBridge()
	self.sub_image = rospy.Subscriber("~image", Image, self.cbImage)
	self.detector = LineDetector()

    def cbImage(self,image_msg):
        image_cv = self.bridge.imgmsg_to_cv2(image_msg, "bgr8")
       
	lines_white = self.detector.detectLines(image_cv, 'white')
	lines_yellow = self.detector.detectLines(image_cv, 'yellow')
	lines_red = self.detector.detectLines(image_cv, 'red') 
	
	rospy.loginfo("[LineDetectorNode] len(lines_white) = %s" %(len(lines_white)))
	rospy.loginfo("[LineDetectorNode] len(lines_yellow) = %s" %(len(lines_yellow)))
	rospy.loginfo("[LineDetectorNode] len(lines_red) = %s" %(len(lines_red)))

    def onShutdown(self):
        rospy.loginfo("[LineDetectorNode] Shutdown.")

if __name__ == '__main__': 
    rospy.init_node('line_detector',anonymous=False)
    line_detector_node = LineDetectorNode()
    rospy.on_shutdown(line_detector_node.onShutdown)
    rospy.spin()

    # resolution = (640, 480)
    # fps = 30

    # # initialize the camera
    # camera = PiCamera()
    # camera.resolution = resolution
    # camera.framerate = fps
    # rawCapture = PiRGBArray(camera, size=resolution)
 
    # # capture frames from the camera
    # for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
    #   bgr = frame.array
        
    #   # Line detector
    #   lines = laneDetector.lineDetector(bgr)
 
    #   # show the frame
    #   cv2.imshow("Frame", bgr)
    #   cv2.waitKey(1)
 
    #   # clear the stream in preparation for the next frame
    #   rawCapture.truncate(0)
