#!/usr/bin/env python
import rospy
import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import CompressedImage,Image
from duckietown_msgs.msg import ObstacleImageDetection, ObstacleImageDetectionList, ObstacleType, Rect

class StaticObjectDetectorTesterNode(object):
    def __init__(self):
        self.node_name = rospy.get_name()
        

        self.pub_fake_images = rospy.Publisher("static_object_detector_node/image_compressed", CompressedImage, queue_size=1)
        rospy.sleep(1)

        #take in directory of images as input, use 01.png, 02.png, etc
        self.im_dir = self.setupParam("~im_dir", "")
        self.num_im = self.setupParam("~num_im", 0)

        self.bridge = CvBridge()
        self.sub_detections = rospy.Subscriber("static_object_detector_node/detection_list", ObstacleImageDetectionList, self.checkDetection)
        #start the first image
        self.num_sent = 0
        rospy.sleep(1)
        self.pubImage()

    def setupParam(self, param_name, default_value):
        value = rospy.get_param(param_name,default_value)
        rospy.set_param(param_name,value) #Write to parameter server for transparancy
        rospy.loginfo("[%s] %s = %s " %(self.node_name,param_name,value))
        return value

    def checkDetection(self, detections):
        #TODO(CL) Do some duckie/cone matching by hand and find image coordinates
        if True:
            rospy.loginfo(detections)
            rospy.loginfo("[%s] Detection %s passed" %(self.node_name,self.num_sent))
        else:
            rospy.loginfo("[%s] Detection %s failed" %(self.node_name,self.num_sent))

        #TODO(CL) After checking the detections list, publish next image
        rospy.sleep(1)
        self.pubImage()

    def pubImage(self):
        if self.num_sent<self.num_im:
            filename = '0'+str(self.num_sent)+'.jpg'
            if self.num_sent>=10:
                filename = str(self.num_sent)+'.jpg'
            image = cv2.imread(self.im_dir+filename)
            msg = CompressedImage()
            msg.header.stamp = rospy.Time.now()
            msg.format = 'jpg'
            msg.data = np.array(cv2.imencode('.jpg', image)[1]).tostring()
            self.pub_fake_images.publish(msg)
            rospy.loginfo("[%s] Image %s published" %(self.node_name,filename))
            self.num_sent+=1

    def onShutdown(self):
        rospy.loginfo("[%s] Shutdown." %(self.node_name))

if __name__ == '__main__':
    rospy.init_node('static_object_detector_tester_node',anonymous=False)
    static_object_detector_tester_node = StaticObjectDetectorTesterNode()
    rospy.on_shutdown(static_object_detector_tester_node.onShutdown)
    rospy.spin()