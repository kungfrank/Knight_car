#!/usr/bin/env python

# numpy and scipy
import numpy as np
#from scipy.ndimage import filters

# OpenCV
import cv2
from cv_bridge import CvBridge, CvBridgeError

#ROS Libraries
#import roslib
import rospy

#ROS Messages
#from std_msgs.msg import String #Imports msg
from sensor_msgs.msg import CompressedImage, Image

bridge = CvBridge()

# Define callback function
def callback(ros_data):
    #print 'received image of type: "%s"' % ros_data.format
    #compressedImage first gets converted into a numpy array
    np_arr = np.fromstring(ros_data.data, np.uint8)
    #decode the image into a raw cv2 image (numpy.ndarray)
    rgb_in = cv2.imdecode(np_arr, cv2.CV_LOAD_IMAGE_COLOR)
    #extract Height in pixels
    H = np_arr[0]
    #extract width in pixels
    W = np_arr[1]
    rgb_out = rgb_in[:, ::-1, :]
    #for u in range(0,H):
    #    for v in range(0,W):
    #        for w in range(0,3):
    #            rgb_out[u, v, w] = rgb_in[u, W - v, w]
    msg = bridge.cv2_to_imgmsg(rgb_out,"bgr8")
    publisher.publish(msg)
    ##### Create CompressedImage ####
    #msg = CompressedImage()
    #msg.header.stamp = rospy.Time.now()
    #msg.format = "jpeg"
    #msg.data = np.array(cv2.imencode('.jpg', rgb_out)[1]).tostring()
    ## Publish new image
    #publisher.publish(msg)	
    #print 'send image of type: "%s"' % msg.format

# Initialize the node with rospy
rospy.init_node('virtual_mirror_node')
# Create publisher
#publisher = rospy.Publisher("~rgb_out",CompressedImage,queue_size=1)
publisher = rospy.Publisher("~rgb_out",Image,queue_size=1)
# Create subscriber
subscriber = rospy.Subscriber("~rgb_in",CompressedImage,callback)
#print 'we made it before the spin'
rospy.spin() #Keeps the script for exiting
