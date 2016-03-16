#!/usr/bin/env python
import numpy as np

import cv2
from cv_bridge import CvBridge, CvBridgeError
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import CompressedImage
from sensor_msgs.msg import Image

bridge = CvBridge()

class image_flipper:

    def __init__(self):
        self.image_pub = rospy.Publisher("~img_out",Image, queue_size = 1)
        #/bill/camera_node/image/compressed
        self.subscriber = rospy.Subscriber("~img_in",CompressedImage, self.callback,  queue_size = 1)

    def callback(self,data):
        np_arr = np.fromstring(data.data,np.uint8)
        image_np = cv2.imdecode(np_arr,cv2.CV_LOAD_IMAGE_COLOR)
        flipped = cv2.flip(image_np,1)
        msg = bridge.cv2_to_imgmsg(flipped,"bgr8")
        self.image_pub.publish(msg)
        #msg = CompressedImage()
        #msg.header.stamp = rospy.Time.now()
        #msg.format = "jpeg"
        #msg.data = np.array(cv2.imencode('.jpg', image_np)[1]).tostring()
        # Publish new image
        #self.image_pub.publish(msg)
   
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