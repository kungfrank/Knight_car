#!/usr/bin/env python
from __future__ import division
import rospy
import cv2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import CompressedImage, Image
import numpy as np
import threading

class ImageAverageNode(object):
    def __init__(self):
        self.node_name = "Image Average"

        # Thread lock 
        self.thread_lock = threading.Lock()

        self.bridge = CvBridge()
      
        # Publishers
        self.pub_image = rospy.Publisher("~average_image", Image, queue_size=1)

        self.cma_image = []
        self.img_seq = 0
       
        # Verbose option 
        #self.verbose = rospy.get_param('~verbose')
        self.verbose = False
        #if self.verbose:
        #    self.toc_pre = rospy.get_time()   

        # Subscribers
        self.sub_image = rospy.Subscriber("~image", CompressedImage, self.cbImage, queue_size=1)
        rospy.loginfo("[%s] Initialized." %(self.node_name))

    def cbImage(self,image_msg):
        # Start a daemon thread to process the image
        thread = threading.Thread(target=self.processImage,args=(image_msg,))
        thread.setDaemon(True)
        thread.start()
        # Returns rightaway

    def processImage(self,image_msg):
        if not self.thread_lock.acquire(False):
            # Return immediately if the thread is locked
            return
        
        # Verbose
        if self.verbose:
            rospy.loginfo("[%s] Latency received = %.3f ms" %(self.node_name, (rospy.get_time()-image_msg.header.stamp.to_sec()) * 1000.0))

        # Decode from compressed image
        # with OpenCV
        image_cv = cv2.imdecode(np.fromstring(image_msg.data, np.uint8), cv2.CV_LOAD_IMAGE_COLOR)

        # Verbose
        if self.verbose:
            self.tic = rospy.get_time()   
            rospy.loginfo("[%s] Latency image decompressed = %.3f ms" %(self.node_name, (self.tic-image_msg.header.stamp.to_sec()) * 1000.0))


        # Process image here
        if (self.img_seq == 0):
            self.cma_image = image_cv
            self.img_seq = 1
        else:
            self.cma_image = self.cma_image + (image_cv - self.cma_image) * (1.0 / (self.img_seq+1))
            self.img_seq += 1
         
        # Publish the frame with lines
        image_msg_out = self.bridge.cv2_to_imgmsg(self.cma_image.astype(np.uint8), "bgr8")
        image_msg_out.header.stamp = image_msg.header.stamp
        self.pub_image.publish(image_msg_out)

        # Verbose
        if self.verbose:
            rospy.loginfo("[%s] Latency sent = %.3f ms" %(self.node_name, (rospy.get_time()-image_msg.header.stamp.to_sec()) * 1000.0))

        # Release the thread lock
        self.thread_lock.release()

    def onShutdown(self):
            rospy.loginfo("[VirtualMirrorNode] Shutdown.")
            

if __name__ == '__main__': 
    rospy.init_node('image_average_tristan_node',anonymous=False)
    image_average_node = ImageAverageNode()
    rospy.on_shutdown(image_average_node.onShutdown)
    rospy.spin()