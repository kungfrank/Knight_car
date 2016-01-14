#!/usr/bin/env python
import rospy
import cv2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from duckietown_msgs.msg import Segment, SegmentList, Pixel
from line_detector.LineDetector import *
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
import time

class LineDetectorNode(object):
    def __init__(self):
        self.node_name = "Line Detector"
        self.bridge = CvBridge()
        self.vert_image_pixels = self.setupParam("~vert_image_pixels",200)
        self.horz_image_pixels = self.setupParam("~horz_image_pixels",320)
        self.top_rows_cutoff   = self.setupParam("~top_rows_cutoff",0)
        self.detector = LineDetector()
#        self.segmentList = SegmentList()
#        self.segment = Segment()
#        self.pixel1 = Pixel()
#        self.pixel2 = Pixel()
#        self.point1 = Point()
#        self.point2 = Point()
        self.sub_image = rospy.Subscriber("~image", Image, self.processImage)
        self.pub_lines = rospy.Publisher("~segment_list", SegmentList, queue_size=1)
        self.pub_image = rospy.Publisher("~image_with_lines",Image, queue_size=1)


    def setupParam(self,param_name,default_value):
        value = rospy.get_param(param_name,default_value)
        rospy.set_param(param_name,value) #Write to parameter server for transparancy
        rospy.loginfo("[%s] %s = %s " %(self.node_name,param_name,value))
        return value


    def processImage(self,image_msg):
        image_cv = self.bridge.imgmsg_to_cv2(image_msg, "bgr8")
        
        # Resize image
        image_cv = cv2.resize(image_cv, (self.horz_image_pixels,self.vert_image_pixels))
        image_cv = image_cv[self.top_rows_cutoff:,:,:]
	
        # Detect lines and normals
        lines_white, normals_white = self.detector.detectLines(image_cv, 'white')
        lines_yellow, normals_yellow = self.detector.detectLines(image_cv, 'yellow')
        lines_red, normals_red = self.detector.detectLines(image_cv, 'red')

        # Draw lines and normals
        image_with_lines_cv = image_cv
        self.detector.drawLines(image_with_lines_cv, lines_white, (0,0,0))
        self.detector.drawLines(image_with_lines_cv, lines_yellow, (255,0,0))
        self.detector.drawLines(image_with_lines_cv, lines_red, (0,255,0))
        self.detector.drawNormals(image_with_lines_cv, lines_white, normals_white)
        self.detector.drawNormals(image_with_lines_cv, lines_yellow, normals_yellow)
        self.detector.drawNormals(image_with_lines_cv, lines_red, normals_red)

        # TODO: Pixel frame to body frame covnersion

        # Add segments to segmentList
        segmentList = SegmentList()
        if len(lines_white)>0:
            rospy.loginfo("[LineDetectorNode] number of white lines = %s" %(len(lines_white)))
            segmentList.segments.extend(self.toSegmentMsg(lines_white, normals_white, Segment.WHITE))
        if len(lines_yellow)>0:
            rospy.loginfo("[LineDetectorNode] number of yellow lines = %s" %(len(lines_yellow)))
            segmentList.segments.extend(self.toSegmentMsg(lines_yellow, normals_yellow, Segment.YELLOW))
        if len(lines_red)>0:
            rospy.loginfo("[LineDetectorNode] number of red lines = %s" %(len(lines_red)))
            segmentList.segments.extend(self.toSegmentMsg(lines_red, normals_red, Segment.RED))
        
        # Publish segmentList
        self.pub_lines.publish(segmentList)
      
        # Publish the frame with lines
        image_msg = self.bridge.cv2_to_imgmsg(image_with_lines_cv, "bgr8")
        self.pub_image.publish(image_msg)

    def onShutdown(self):
            rospy.loginfo("[LineDetectorNode] Shutdown.")
            

    def toSegmentMsg(self,  lines, normals, color):
        
        segmentMsgList = []
        for u1,v1,u2,v2,norm_u,norm_v in np.hstack((lines,normals)):
            pixel1 = Pixel()
            pixel2 = Pixel()
            pixel1.u = int(u1)
            pixel1.v = int(v1)
            pixel2.u = int(u2)
            pixel2.v = int(v2)
            
            segment = Segment()
            segment.color = color
            segment.pixels[0] = pixel1
            segment.pixels[1] = pixel2
            segment.normal_u = norm_u
            segment.normal_v = norm_v
             
            segmentMsgList.append(segment)
            # TODO: assign segment.points
        return segmentMsgList


if __name__ == '__main__': 
    rospy.init_node('line_detector',anonymous=False)
    line_detector_node = LineDetectorNode()
    rospy.on_shutdown(line_detector_node.onShutdown)
    rospy.spin()

