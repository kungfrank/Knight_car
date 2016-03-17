#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Image, CompressedImage
from duckietown_msg_rkk.msg import FlipDirection
import numpy as np
from cv_bridge import CvBridge
import cv2

# Virtual Mirror Tester Node tests functionality of virtual_mirror_rkk_node
class VirtualMirrorTesterNode(object):
    def __init__(self):
        self.node_name = 'Virtual Mirror RKK Tester'
        self.flip_direction = self.setupParam("virtual_mirror_rkk_node/flip_direction", 'horz')
        self.orig_path = self.setupParam("~orig_path", '../test_images/lenna.png')
        self.horz_path = self.setupParam("~horz_path", '../test_images/lenna_horz.png')
        self.vert_path = self.setupParam("~vert_path", '../test_images/lenna_vert.png')

        self.sub_raw = rospy.Subscriber("~rgb_in", Image, self.cbCheckImage)
        self.sub_comp = rospy.Publisher("~rgb_out", CompressedImage, queue_size=1, latch=True)
        self.bridge = CvBridge()
        self.result_vert = None
        self.result_horz = None

        img = cv2.imread(self.orig_path)
        self.image_orig_msg = CompressedImage()
        self.image_orig_msg.header.stamp = rospy.Time.now()
        self.image_orig_msg.format = "png"
        self.image_orig_msg.data = np.array(cv2.imencode('.png', img)[1]).tostring()

        self.sub_comp.publish(self.image_orig_msg)
        rospy.loginfo("%s has started", self.node_name)

    def verifyResult(self):
        if self.result_vert is True and self.result_horz is True:
            rospy.loginfo("[%s] === All Tests Passed === " %(self.node_name))
        elif self.result_vert is False or self.result_horz is False:
            rospy.logwarn("[%s] Test Failed! " %(self.node_name))
        else:
            new_dir = 'horz' if self.flip_direction == 'vert' else 'vert'
            rospy.set_param('virtual_mirror_rkk_node/flip_direction', new_dir)
            rospy.loginfo("[%s] %s Test completed. Starting %s test." %(self.node_name, self.code2Word(self.flip_direction), self.code2Word(new_dir)))
            self.flip_direction = new_dir
            rospy.sleep(2.) #Must wait longer than the polling rate of the virutal_mirror_node
            self.sub_comp.publish(self.image_orig_msg)

    def cbCheckImage(self, image_msg):
        rospy.loginfo("[%s] %s image received from virtual mirror" %(self.node_name,self.code2Word(self.flip_direction)))
        image_flip = self.bridge.imgmsg_to_cv2(image_msg)
        if self.flip_direction == 'horz':
            image_flip_ground = cv2.imread(self.horz_path)
            self.result_horz = np.array_equal(image_flip, image_flip_ground)
        else:
            image_flip_ground = cv2.imread(self.vert_path)
            self.result_vert = np.array_equal(image_flip, image_flip_ground)

        self.verifyResult()

    def code2Word(self, code):
        return "Horizontal" if code == 'horz' else 'Vertical'

    def setupParam(self,param_name,default_value):
        value = rospy.get_param(param_name,default_value)
        rospy.set_param(param_name,value) #Write to parameter server for transparancy
        rospy.loginfo("[%s] %s = %s " %(self.node_name,param_name,value))
        return value

if __name__ == '__main__':
    rospy.init_node('virtual_mirror_rkk_tester', anonymous=False)
    virtual_mirror_tester_node = VirtualMirrorTesterNode()
    rospy.spin()