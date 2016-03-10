#!/usr/bin/env python
import rospy
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
from sensor_msgs.msg import CompressedImage,Image
import time
import io
# bridge = CvBridge()
# publisher = None

class AverageImageNode(object):
    def __init__(self):
        self.node_name = rospy.get_name()
        self.bridge = CvBridge()
        
        # self.publish_freq = self.setupParam("~publish_freq",1.0)
        # self.publish_duration = rospy.Duration.from_sec(1.0/self.publish_freq)
        self.pub_raw = rospy.Publisher("~rgb_out",Image,queue_size=1)
        # self.last_stamp = rospy.Time.now()        
        self.sub_compressed_img = rospy.Subscriber("~rgb_in",CompressedImage,self.cbImg,queue_size=1)
        # self.stream = io.BytesIO()
        self.count = 0.
        self.average_img = None


    def avImg(self, np_img):
        if self.count == 0.:
            self.average_img = np_img
            self.count = 1.
        else:
            aux = self.count + 1.
            # print self.count/aux, 1./aux
            self.average_img = cv2.addWeighted(self.average_img, self.count/aux, np_img, (1-self.count/aux), 0.0)
            
            self.count += 1.
        return self.average_img

        # if self.count == 0:
        #     self.average_img = np_img
        #     self.count = 1.
        # else:
        #     aux = self.count + 1.
        #     new_average_img = self.average_img *self.count + np_img 
        #     self.average_img = new_average_img/aux
        #     self.count += 1.
        # return self.average_img

    def cbImg(self, msg):
        np_arr = np.fromstring(msg.data, np.uint8)
        
        cv_image = cv2.imdecode(np_arr, cv2.CV_LOAD_IMAGE_COLOR)
        # time_1 = time.time()
        av_image = self.avImg(cv_image.astype('float'))
        
        img_msg = self.bridge.cv2_to_imgmsg(av_image.astype('uint8'), "bgr8")
        # time_2 = time.time()
        img_msg.header.stamp = msg.header.stamp
        img_msg.header.frame_id = msg.header.frame_id
        self.pub_raw.publish(img_msg)


if __name__ == '__main__': 
    rospy.init_node('averageimg',anonymous=False)
    node = AverageImageNode()
    rospy.spin()


