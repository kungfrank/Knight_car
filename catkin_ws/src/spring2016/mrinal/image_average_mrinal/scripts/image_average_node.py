#!/usr/bin/env python
import sys
import rospy
import cv2
import numpy as np
from std_msgs.msg import String
from sensor_msgs.msg import CompressedImage

class image_converter:

  def __init__(self):
    self.image_sub = rospy.Subscriber("rgb_in",CompressedImage,self.callback)
    self.image_pub = rospy.Publisher("rgb_out/compressed",CompressedImage,queue_size=1)
    self.totframes = 0

  def callback(self,ros_data):
    rospy.loginfo('[INFO] Received Image, totframes=%d'%self.totframes)
    np_arr = np.fromstring(ros_data.data, np.uint8)
    image_np = cv2.imdecode(np_arr, cv2.CV_LOAD_IMAGE_COLOR)

    #### Image Processing ####
    if not hasattr(self,'image_avg'):
        rospy.loginfo('[INFO] Initializing image_avg')
        self.image_avg = np.float64(image_np)
        self.totframes = 1.
    else:
        cv2.accumulateWeighted(image_np, self.image_avg, 1./(self.totframes+1))
        self.totframes += 1.

    #### Create CompressedImage ####
    msg = CompressedImage()
    msg.header.stamp = rospy.Time.now()
    msg.format = "jpeg"
    msg.data = np.array(cv2.imencode('.jpg', cv2.convertScaleAbs(self.image_avg))[1]).tostring()
    self.image_pub.publish(msg)

def main(args):
  rospy.init_node('image_average_mrinal', anonymous=True)
  ic = image_converter()
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")

if __name__ == '__main__':
    main(sys.argv)