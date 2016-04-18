#!/usr/bin/env python
import numpy as np

import cv2
from cv_bridge import CvBridge, CvBridgeError
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import CompressedImage
from sensor_msgs.msg import Image
import time
import io
from duckietown_msg_araki.msg import FlipDirection

class image_flipper:

    def __init__(self):
        self.node_name = rospy.get_name()
        self.bridge = CvBridge()

        self.image_pub = rospy.Publisher("~img_out",Image, queue_size = 1)
        #/bill/camera_node/image/compressed
        self.subscriber = rospy.Subscriber("~img_in",CompressedImage, self.callback,  queue_size = 1)

        self.param_pub = rospy.Publisher("~flip_direction",FlipDirection,queue_size=1)
        self.param_msg = FlipDirection()
        self.flip_direction = self.setupParam("~flip_direction","horz")
        self.param_timer = rospy.Timer(rospy.Duration.from_sec(1.0),self.cbParamTimer)


    def callback(self,data):
        np_arr = np.fromstring(data.data,np.uint8)
        image_np = cv2.imdecode(np_arr,cv2.CV_LOAD_IMAGE_COLOR)
        if self.flip_direction == "horz":
            flipped = cv2.flip(image_np,1)
        elif self.flip_direction == "vert":
            flipped = cv2.flip(image_np,0)
        else:
            raise Exception('invalid parameter')

        msg = self.bridge.cv2_to_imgmsg(flipped,"bgr8")
        self.image_pub.publish(msg)
        #msg = CompressedImage()
        #msg.header.stamp = rospy.Time.now()
        #msg.format = "jpeg"
        #msg.data = np.array(cv2.imencode('.jpg', image_np)[1]).tostring()
        # Publish new image
        #self.image_pub.publish(msg)

    def cbParamTimer(self,event):
        #self.flip_direction = self.setupParam("~flip_direction", "horz")
        self.flip_direction = rospy.get_param("~flip_direction")
        if self.flip_direction == 'horz':
            self.param_msg.flip_direction = FlipDirection.HORZ
        else:
            self.param_msg.flip_direction = FlipDirection.VERT
        self.param_pub.publish(self.param_msg)

    def setupParam(self,param_name,default_value):
        value = rospy.get_param(param_name,default_value)
        rospy.set_param(param_name,value) #Write to parameter server for transparancy
        rospy.loginfo("[%s] %s = %s " %(self.node_name,param_name,value))
        return value
   
def virtual_mirror_araki_node():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # node are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('virtual_mirror_araki_node', anonymous=True)
    flipper = image_flipper()

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print "Shutting down ROS Image feature detector module"
    cv2.destroyAllWindows()


if __name__ == '__main__':
    virtual_mirror_araki_node()