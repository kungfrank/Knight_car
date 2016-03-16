#!/usr/bin/env python
import rospy
import cv2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import CompressedImage, Image
from duckietown_msg_joewl.msg import FlipDirection
import numpy as np

class VirtualMirrorTester(object):
    def __init__(self):
        # Save the name of the node
        self.node_name = rospy.get_name()
        
        rospy.loginfo("[%s] Initialzing." %(self.node_name))

	    # Initialize CV bridge
        self.bridge = CvBridge()
        
        # Initialize tester state variables
        self.sent_image_count = 1
        self.received_horz_image = False
        self.received_vert_image = False
        self.tests_complete = False

        # Setup parameters
        #self.flip_direction = self.setupParam("~flip_direction", "horz")
        self.pub_timestep = self.setupParam("~pub_timestep", 1.0)
        self.test_image_path = self.setupParam("~test_image_path", "")
        self.test_image_format = self.setupParam("~test_image_format", "")
        self.test_image_count = self.setupParam("~test_image_count", 0)

        # Setup publishers
        self.pub_test_image = rospy.Publisher("virtual_mirror_joewl_node/image_in", CompressedImage, queue_size=1)

        # Setup subscriber
        self.sub_processed_image = rospy.Subscriber("virtual_mirror_joewl_node/image_out", Image, self.checkImage)
        self.sub_flip_direction = rospy.Subscriber("virtual_mirror_joewl_node/flip_direction", FlipDirection, self.cbFlipDirection)

        rospy.loginfo("[%s] Initialzed." %(self.node_name))
        
        # Timers
        # self.pub_timer = rospy.Timer(rospy.Duration.from_sec(self.pub_timestep), self.cbPubTimer)

    def setupParam(self,param_name,default_value):
        value = rospy.get_param(param_name,default_value)
        rospy.set_param(param_name,value) #Write to parameter server for transparancy
        rospy.loginfo("[%s] %s = %s " %(self.node_name,param_name,value))
        return value

    def cbFlipDirection(self,direction_msg):
        if not self.tests_complete:
            self.flip_direction = direction_msg.direction

            if self.received_horz_image and self.received_vert_image:
                if self.sent_image_count < self.test_image_count:
                    self.sent_image_count += 1
                    self.received_horz_image = False
                    self.received_vert_image = False
                else:
                    #self.sent_image_count = 1
                    rospy.loginfo("[%s] Tests complete." %(self.node_name))
                    self.tests_complete = True
       
            if not self.received_horz_image:
                if self.flip_direction != "horz":
                    rospy.set_param("virtual_mirror_joewl_node/flip_direction","horz")
                else:
                    self.sendTestImage()
            elif not self.received_vert_image:
                if self.flip_direction != "vert":
                    rospy.set_param("virtual_mirror_joewl_node/flip_direction","vert")
                else:
                    self.sendTestImage()

    def sendTestImage(self):
        img_name = '0'+str(self.sent_image_count)+'_orig.'+self.test_image_format
        test_img = cv2.imread(self.test_image_path+img_name)
        msg = CompressedImage()
        msg.header.stamp = rospy.Time.now()
        msg.format = self.test_image_format
        msg.data = np.array(cv2.imencode('.'+self.test_image_format, test_img)[1]).tostring()
        self.pub_test_image.publish(msg)


    def checkImage(self,image_msg):
	    # Convert image message to CV image:
        image_cv = self.bridge.imgmsg_to_cv2(image_msg, "bgr8")

	    # F
        if self.flip_direction == "horz":
            img_name = '0'+str(self.sent_image_count)+'_horz.'+self.test_image_format
            self.received_horz_image = True
        elif self.flip_direction == "vert":
            img_name = '0'+str(self.sent_image_count)+'_vert.'+self.test_image_format
            self.received_vert_image = True

        image_flipped = cv2.imread(self.test_image_path+img_name)
        if np.array_equal(image_cv,image_flipped):
            rospy.loginfo("[%s] Image %s %s passed" %(self.node_name,self.sent_image_count,self.flip_direction))
        else:
            rospy.loginfo("[%s] Image %s %s failed" %(self.node_name,self.sent_image_count,self.flip_direction))

    def on_shutdown(self):
        rospy.loginfo("[%s] Shutting down." %(self.node_name))

if __name__ == '__main__':
    # Initialize the node with rospy
    rospy.init_node('virtual_mirror_joewl_tester', anonymous=False)

    # Create the NodeName object
    virtual_mirror_joewl_tester_node = VirtualMirrorTester()

    # Setup proper shutdown behavior 
    rospy.on_shutdown(virtual_mirror_joewl_tester_node.on_shutdown)
    
    # Keep it spinning to keep the node alive
    rospy.spin()
