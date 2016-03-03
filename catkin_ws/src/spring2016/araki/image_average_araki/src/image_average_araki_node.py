#!/usr/bin/env python
import numpy as np

import cv2, sys
from cv_bridge import CvBridge, CvBridgeError
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import CompressedImage
from sensor_msgs.msg import Image

bridge = CvBridge()

class image_averager:

    def __init__(self):
        self.image_pub = rospy.Publisher("~img_out",Image, queue_size = 1)
        #/bill/camera_node/image/compressed
        self.subscriber = rospy.Subscriber("~img_in",CompressedImage, self.callback,  queue_size = 1)
        self.average = None
        self.num = 0

    def callback(self,data):
        try:
            np_arr = np.fromstring(data.data,np.uint8)
            image_np = cv2.imdecode(np_arr,cv2.CV_LOAD_IMAGE_COLOR)
            new_im = image_np.astype('float32')
            rgb_out_32 = new_im
        except CvBridgeError as e:
            print(e)
        # (N/(N+1))*Old + (1/(N+1))*New = Average
        alpha = (self.num/(self.num + 1.))
        beta = 1. - alpha


        if self.num == 0:
            self.average = new_im

        cv2.addWeighted(self.average,alpha,new_im,beta,0.0,rgb_out_32)
        self.num += 1
        rgb_out = rgb_out_32.astype('uint8')
        self.average = rgb_out_32

        msg = bridge.cv2_to_imgmsg(rgb_out,"bgr8")
        try:
            self.image_pub.publish(msg)
        except CvBridgeError as e:
            print(e)
        
        #msg = CompressedImage()
        #msg.header.stamp = rospy.Time.now()
        #msg.format = "jpeg"
        #msg.data = np.array(cv2.imencode('.jpg', image_np)[1]).tostring()
        # Publish new image
        #self.image_pub.publish(msg)
   
def image_average_araki_node():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # node are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('virtual_mirror_araki_node', anonymous=True)
    averager = image_averager()

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print "Shutting down ROS Image feature detector module"
    cv2.destroyAllWindows()


if __name__ == '__main__':
    image_average_araki_node()