#!/usr/bin/env python
import rospy
import cv2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import CompressedImage, Image
from duckietown_msgs_tristan.msg import FlipDirection
import numpy as np
import threading

class VirtualMirrorNode(object):
    def __init__(self):
        self.node_name = "Virtual Mirror"
        
        self.flip_direction = self.setupParam("~flip_direction","vert")

        self.param_timer = rospy.Timer(rospy.Duration.from_sec(1.0),self.cbParamTimer)

        # Thread lock 
        self.thread_lock = threading.Lock()

        self.bridge = CvBridge()
      
        # Publishers
        self.pub_image = rospy.Publisher("~mirror_image", Image, queue_size=1)
        self.pub_flip_direction = rospy.Publisher("~flip_direction", FlipDirection, queue_size=1)
       
        # Verbose option 
        #self.verbose = rospy.get_param('~verbose')
        self.verbose = False
        #if self.verbose:
        #    self.toc_pre = rospy.get_time()   

        # Subscribers
        self.sub_image = rospy.Subscriber("~image", CompressedImage, self.cbImage, queue_size=1)
        rospy.loginfo("[%s] Initialized." %(self.node_name))

    def cbParamTimer(self,event):
        self.flip_direction = rospy.get_param("~flip_direction", "vert")
        flip_out = FlipDirection()
        flip_out.flip_direction = self.flip_direction
        self.pub_flip_direction.publish(flip_out)


    def setupParam(self,param_name,default_value):
        value = rospy.get_param(param_name,default_value)
        rospy.set_param(param_name,value) #Write to parameter server for transparancy
        rospy.loginfo("[%s] %s = %s " %(self.node_name,param_name,value))
        return value

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
        if self.flip_direction == "horz":
            mirrorImage = image_cv[:,::-1,:]
        else:
            mirrorImage = image_cv[::-1,:,:]
         
        # Publish the frame with lines
        image_msg_out = self.bridge.cv2_to_imgmsg(mirrorImage, "bgr8")
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
    rospy.init_node('virtual_mirror_tristan_node',anonymous=False)
    virtual_mirror_node = VirtualMirrorNode()
    rospy.on_shutdown(virtual_mirror_node.onShutdown)
    rospy.spin()