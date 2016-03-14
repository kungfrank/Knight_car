#!/usr/bin/env python
import rospy
import cv2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import CompressedImage, Image
from duckietown_msg_joewl.msg import FlipDirection
import numpy as np

class VirtualMirror(object):
    def __init__(self):
        # Save the name of the node
        self.node_name = rospy.get_name()
        
        rospy.loginfo("[%s] Initialzing." %(self.node_name))

	    # Initialize CV bridge
        self.bridge = CvBridge()

        # Setup parameters
        self.flip_direction = self.setupParam("~flip_direction", "horz")
        self.pub_timestep = self.setupParam("~pub_timestep", 1.0)

        # Setup publishers
        self.pub_image_out = rospy.Publisher("~image_out", Image, queue_size=1)
        self.pub_flip_direction = rospy.Publisher("~flip_direction", FlipDirection, queue_size=1)

        # Setup subscriber
        self.sub_image_in = rospy.Subscriber("~image_in", CompressedImage, self.flipImage)

        rospy.loginfo("[%s] Initialzed." %(self.node_name))
        
        # Timers
        self.param_timer = rospy.Timer(rospy.Duration.from_sec(1.0),self.cbParamTimer)
        self.pub_timer = rospy.Timer(rospy.Duration.from_sec(self.pub_timestep), self.cbPubTimer)

    def cbParamTimer(self,event):
        self.flip_direction = rospy.get_param("~flip_direction", "horz")

    def cbPubTimer(self,event):
        direction_msg = FlipDirection()
        direction_msg.direction = self.flip_direction
        self.pub_flip_direction.publish(direction_msg)

    def setupParam(self,param_name,default_value):
        value = rospy.get_param(param_name,default_value)
        rospy.set_param(param_name,value) #Write to parameter server for transparancy
        rospy.loginfo("[%s] %s = %s " %(self.node_name,param_name,value))
        return value

    def flipImage(self,image_msg):
	    # Convert image message to CV image:
	    #image_cv = self.bridge.imgmsg_to_cv2(image_msg, "bgr8")
        image_cv = cv2.imdecode(np.fromstring(image_msg.data, np.uint8), cv2.CV_LOAD_IMAGE_COLOR)

	    # Flip image based on flip_direction parameter:
        if self.flip_direction == "horz":	    
            image_flipped = cv2.flip(image_cv,1)
        elif self.flip_direction == "vert":
            image_flipped = cv2.flip(image_cv,0)

	    # Convert flipped image to image message:
        image_msg_out = self.bridge.cv2_to_imgmsg(image_flipped, "bgr8")
        image_msg_out.header.stamp = image_msg.header.stamp

	    # Publish flipped image:
        self.pub_image_out.publish(image_msg_out)

    def on_shutdown(self):
        rospy.loginfo("[%s] Shutting down." %(self.node_name))

if __name__ == '__main__':
    # Initialize the node with rospy
    rospy.init_node('virtual_mirror_joewl', anonymous=False)

    # Create the NodeName object
    virtual_mirror_joewl_node = VirtualMirror()

    # Setup proper shutdown behavior 
    rospy.on_shutdown(virtual_mirror_joewl_node.on_shutdown)
    
    # Keep it spinning to keep the node alive
    rospy.spin()
