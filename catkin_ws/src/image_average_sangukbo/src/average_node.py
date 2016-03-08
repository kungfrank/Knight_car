#!/usr/bin/env python
import sys
import rospy
import cv2
import numpy as np
from std_msgs.msg import String
from sensor_msgs.msg import CompressedImage

class image_converter:

  def __init__(self):
    self.image_sub = rospy.Subscriber("/ferrari/camera_node/image/compressed",CompressedImage,self.callback)
    self.image_pub = rospy.Publisher("average_compressed/compressed",CompressedImage,queue_size=1)
    self.totframes = 0

  def callback(self,ros_data):
    rospy.loginfo('[INFO] Received Image, totframes=%d'%self.totframes)
    np_arr = np.fromstring(ros_data.data, np.uint8)
    image_np = cv2.imdecode(np_arr, cv2.CV_LOAD_IMAGE_COLOR)

    #### Image Processing ####
    if not hasattr(self,'average_image'):
        rospy.loginfo('[INFO] Initializing average_image')
        self.average_image = np.float64(image_np)
        self.totframes = 1.
    else:
        cv2.accumulateWeighted(image_np, self.average_image, 1./(self.totframes+1))
        self.totframes += 1.

    #### Create CompressedImage ####
    msg = CompressedImage()
    msg.header.stamp = rospy.Time.now()
    msg.format = "jpeg"
    msg.data = np.array(cv2.imencode('.jpg', cv2.convertScaleAbs(self.average_image))[1]).tostring()
    self.image_pub.publish(msg)

def main(args):
  rospy.init_node('average_node', anonymous=True)
  ic = image_converter()
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")

if __name__ == '__main__':
    main(sys.argv)
