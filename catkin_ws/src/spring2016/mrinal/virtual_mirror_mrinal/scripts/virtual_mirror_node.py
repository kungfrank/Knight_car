#!/usr/bin/env python
import sys
import rospy
import cv2
import numpy as np
from std_msgs.msg import String
from sensor_msgs.msg import CompressedImage

class image_converter:

  def __init__(self):
    self.image_sub = rospy.Subscriber("image_sub",CompressedImage,self.callback)
    self.image_pub = rospy.Publisher("image_pub/compressed",CompressedImage,queue_size=1)

  def callback(self,ros_data):
    np_arr = np.fromstring(ros_data.data, np.uint8)
    image_np = cv2.imdecode(np_arr, cv2.CV_LOAD_IMAGE_COLOR)
    image_np = cv2.flip(image_np, 1)

    #### Create CompressedIamge ####
    msg = CompressedImage()
    msg.header.stamp = rospy.Time.now()
    msg.format = "jpeg"
    msg.data = np.array(cv2.imencode('.jpg', image_np)[1]).tostring()
    self.image_pub.publish(msg)

def main(args):
  ic = image_converter()
  rospy.init_node('virtual_mirror_mrinal', anonymous=True)
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")

if __name__ == '__main__':
    main(sys.argv)