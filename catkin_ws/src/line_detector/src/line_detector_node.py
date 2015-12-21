#!/usr/bin/env python
import rospy
import cv2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from duckietown_msgs.msg import Segment, SegmentList, Pixel, Point3
from LineDetector import *
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
import time

# from picamera.array import PiRGBArray
# from picamera import PiCamera

class LineDetectorNode(object):
    def __init__(self):
        self.bridge = CvBridge()
	self.sub_image = rospy.Subscriber("~image", Image, self.cbImage)
	# self.pub_lines = rospy.Publisher("segments", Segments)
        # self.marker_pub = rospy.Publisher("~visualization_marker",Marker)
	self.image_pub = rospy.Publisher("~image_with_lines",Image, queue_size=1);
	self.detector = LineDetector()
        # self.white_line_list=Marker()
        # self.yellow_line_list=Marker()
        # self.red_line_list=Marker()
        # markerSetup();


    def cbImage(self,image_msg):
        image_cv = self.bridge.imgmsg_to_cv2(image_msg, "bgr8")
	#rospy.loginfo("Type of image: %s" %str(image_cv.shape))
	
	lines_white = self.detector.detectLines(image_cv, 'white')
	lines_yellow = self.detector.detectLines(image_cv, 'yellow')
	lines_red = self.detector.detectLines(image_cv, 'red')

	image_with_lines_cv = image_cv
	self.detector.drawLines(image_with_lines_cv, lines_white, (0,0,0))
	self.detector.drawLines(image_with_lines_cv, lines_yellow, (255,0,0))
	self.detector.drawLines(image_with_lines_cv, lines_red, (0,255,0))

	#rospy.loginfo("Type of image: %s" %str(image_with_lines_cv.shape))			
	image_msg = self.bridge.cv2_to_imgmsg(image_with_lines_cv, "bgr8")
	self.image_pub.publish(image_msg)
	
	if len(lines_white)>0:	
		rospy.loginfo("[LineDetectorNode] number of white lines = %s" %(len(lines_white)))
	if len(lines_yellow)>0:
		rospy.loginfo("[LineDetectorNode] number of yellow lines = %s" %(len(lines_yellow)))
	if len(lines_red)>0:
		rospy.loginfo("[LineDetectorNode] number of red lines = %s" %(len(lines_red)))
        # publishSegmentList()

    def onShutdown(self):
        rospy.loginfo("[LineDetectorNode] Shutdown.")
"""  
def markerSetup(self):
	white_line_list = 
        white_line_list.id=0
        yellow_line_list.id=1
        red_line_list.id=2
        white_line_list.type = Marker.LINE_LIST
        yellow_line_list.type = Marker.LINE_LIST
        red_line_list.type = Marker.LINE_LIST
        white_line_list.scale.x=0.2
        yellow_line_list.scale.x=0.2
        red_line_list.scale.x=0.2
        white_line_list.color.r=1.0
        white_line_list.color.g=1.0
        white_line_list.color.b=1.0
        white_line_list.color.a=1.0
        yellow_line_list.color.g=1.0
        yellow_line_list.color.r=1.0
        yellow_line_list.color.a=1.0
        red_line_list.color.r=1.0
        red_line_list.color.a=1.0

    def publishSegmentList(self,lines_white,lines_yellow,lines_red):
        for x1,y1,x2,y2 in lines_white:
            point1=Point()
            point2=Point()
            point1.x=x1
            point1.y=y1
            point2.x=x2
            point2.y=y2
            white_line_list.push_back(point1)
            white_line_list.push_back(point2)
            ..
            

        for x1  lines_yello
		..
        
        self.marker_pub.Publish(lines_white)
        self.marker_pub.Publish(lines_yellow)
        ...
"""   

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
