#!/usr/bin/env python
import rospy
import cv2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from duckietown_msgs.msg import Segment, SegmentList, Pixel
from LineDetector import *
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
import time

class LineDetectorNode(object):
    def __init__(self):
        self.bridge = CvBridge()

        self.sub_image = rospy.Subscriber("~image", Image, self.cbImage)
        self.pub_lines = rospy.Publisher("~segmentList", SegmentList, queue_size=1)
        self.pub_image = rospy.Publisher("~image_with_lines",Image, queue_size=1)

        self.detector = LineDetector()

        self.segmentList = SegmentList()
        self.segment = Segment()

    def cbImage(self,image_msg):
        image_cv = self.bridge.imgmsg_to_cv2(image_msg, "bgr8")
	
        # Line detection
        lines_white = self.detector.detectLines(image_cv, 'white')
        lines_yellow = self.detector.detectLines(image_cv, 'yellow')
        lines_red = self.detector.detectLines(image_cv, 'red')

        # Draw detected lines
        image_with_lines_cv = image_cv
        self.detector.drawLines(image_with_lines_cv, lines_white, (0,0,0))
        self.detector.drawLines(image_with_lines_cv, lines_yellow, (255,0,0))
        self.detector.drawLines(image_with_lines_cv, lines_red, (0,255,0))
        
        # TODO: Pixel frame to body frame covnersion

        # Publish the segments
        if len(lines_white)>0:
            self.publishSegmentList(lines_white, self.segmentList.WHITE)	
            rospy.loginfo("[LineDetectorNode] number of white lines = %s" %(len(lines_white)))
        if len(lines_yellow)>0:
            self.publishSegmentList(lines_yellow, self.segmentList.YELLOW)	
            rospy.loginfo("[LineDetectorNode] number of yellow lines = %s" %(len(lines_yellow)))
        if len(lines_red)>0:
            self.publishSegmentList(lines_red, self.segmentList.RED)	
            rospy.loginfo("[LineDetectorNode] number of red lines = %s" %(len(lines_red)))
        
        # Publish the frame with lines
        image_msg = self.bridge.cv2_to_imgmsg(image_with_lines_cv, "bgr8")
        self.pub_image.publish(image_msg)

    def onShutdown(self):
            rospy.loginfo("[LineDetectorNode] Shutdown.")
    
    def publishSegmentList(self, lines, color):
        self.segmentList.segments = []
        self.segmentList.color = color
        pixel1=Pixel()
        pixel2=Pixel()
        point1=Point()
        point2=Point()

        for u1,v1,u2,v2 in lines:
            pixel1.u=u1
            pixel1.v=v1
            pixel2.u=u2
            pixel2.v=v2
            self.segment.pixels[0]=pixel1
            self.segment.pixels[1]=pixel2
                
            # TODO: assign segment.points
        
            self.segmentList.segments.append(self.segment)

        self.pub_lines.publish(self.segmentList)
          

if __name__ == '__main__': 
    rospy.init_node('line_detector',anonymous=False)
    line_detector_node = LineDetectorNode()
    rospy.on_shutdown(line_detector_node.onShutdown)
    rospy.spin()

