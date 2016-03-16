#!/usr/bin/env python
import sys
import cv2
import rospy
from sensor_msgs.msg import CompressedImage,Image
from std_msgs.msg import String
import numpy

rospy.init_node('mirror_node')
publisher = rospy.Publisher("mirror_compressed/compressed", CompressedImage, queue_size=1)
def virtual_mirror(image):
    
    rgb_in_temp = numpy.fromstring(image.data, numpy.uint8)
    rgb_in = cv2.imdecode(rgb_in_temp, cv2.CV_LOAD_IMAGE_COLOR)
    rgb_out = cv2.flip(rgb_in, 1)
#    V = len(rgb_in[0][0])
#    W = len(rgb_in[0])
#    H = len(rgb_in)
#    rgb_out = numpy.zeros((H, W, V))
#    for i in range(0, H):
#        for k in range(0, V):
#            for j in range(0, W):
#                rgb_out[i][j][k] = rgb_in[i][W-1-j][k]
    msg = CompressedImage()
    msg.header.stamp = rospy.Time.now()
    msg.format = "jpeg"
    msg.data = numpy.array(cv2.imencode('.jpg', rgb_out)[1]).tostring()
    publisher.publish(msg)
subscriber = rospy.Subscriber("/setlist/camera_node/image/compressed", CompressedImage, virtual_mirror)
rospy.spin()
