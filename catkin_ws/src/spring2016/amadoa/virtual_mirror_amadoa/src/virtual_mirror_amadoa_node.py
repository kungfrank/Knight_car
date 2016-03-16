#!/usr/bin/env python
import rospy
import numpy as np
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import CompressedImage
from duckietown_msg_amadoa.msg import Flip
from numpy import array
import time


class MirrorNode(object):
    def __init__(self):
        self.node_name = "virtual_mirror_amadoa_node"
        self.flip_direction = self.setupParam("~flip_direction", "horz") # horz or vert

        self.subscriber = rospy.Subscriber("~duckiebot_image", CompressedImage, self.imageCallback)

        self.pub_mirror = rospy.Publisher("~mirrored_image",CompressedImage,queue_size=1)

        self.pub_flip = rospy.Publisher("~flip", Flip, queue_size=1)

        self.param_timer = rospy.Timer(rospy.Duration.from_sec(1.0),self.cbParamTimer)

        # rospy.loginfo("virtual_mirror_amadoa_node initialized")

    def setupParam(self, param_name,default_value):
        value = rospy.get_param(param_name,default_value)
        rospy.set_param(param_name,value) #Write to parameter server for transparancy
        rospy.loginfo("[%s] %s = %s " %(self.node_name, param_name,value))
        return value

    def imageCallback(self, msg):
        #### direct conversion to CV2 ####
        # print 'image received'
        np_arr = np.fromstring(msg.data, np.uint8)
        image_np = cv2.imdecode(np_arr, cv2.CV_LOAD_IMAGE_COLOR)

        flipCode = 1 if self.flip_direction == "horz" else 0
        image_np_out = cv2.flip(image_np, flipCode)

        #### Create CompressedIamge ####
        out = CompressedImage()
        out.header.stamp = rospy.Time.now()
        out.format = "png"
        np_arr_out = cv2.imencode('.png', image_np_out)[1]
        out.data = np.array(np_arr_out).tostring()

        # Publish new image
        self.pub_mirror.publish(out)

    def cbParamTimer(self,event):
        self.flip_direction = rospy.get_param("~flip_direction", "horz")
        flip_msg = Flip()
        flip_msg.flip = 1 if self.flip_direction == "horz" else 0
        self.pub_flip.publish(flip_msg)

    def onShutdown(self):
        rospy.loginfo("[virtual_mirror_amadoa_node] Shutdown.")


if __name__ == '__main__':
    rospy.init_node("virtual_mirror_amadoa_node",anonymous=False)
    mirror_node = MirrorNode()
    rospy.on_shutdown(mirror_node.onShutdown)
    rospy.spin()
