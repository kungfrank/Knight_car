#!/usr/bin/env python
import sys
import rospy
import cv2
import numpy as np
from std_msgs.msg import String
from sensor_msgs.msg import CompressedImage
from duckietown_msg_mrinal.msg import FlipStyle

class image_processor:

  def __init__(self):
    self.node_name = rospy.get_name()
    self.image_sub = rospy.Subscriber("image_sub",CompressedImage,self.callback)
    self.image_pub = rospy.Publisher("image_pub/compressed",CompressedImage, queue_size=1)
    self.config_pub = rospy.Publisher("flip_direction", FlipStyle, queue_size=1)

    #### Setup Flip Parameter ####
    self.flipstyle = FlipStyle()
    self.flip_string = self.setupParam("~flip_direction", 'horz')
    self.update_flip_style(self.flip_string)

    #### Setup Parameter and Publisher Timers ####
    self.param_timer = rospy.Timer(rospy.Duration.from_sec(1.0),self.cbParamTimer)
    self.pub_timer = rospy.Timer(rospy.Duration.from_sec(1.0),self.cbPubTimer)

  def setupParam(self,param_name,default_value):
    value = rospy.get_param(param_name,default_value)
    rospy.set_param(param_name,value) #Write to parameter server for transparancy
    rospy.loginfo("[%s] %s = %s " %(self.node_name,param_name,value))
    return value

  def cbParamTimer(self,event):
    self.flip_string = rospy.get_param("~flip_direction", 1.0)
    self.update_flip_style(self.flip_string)

  def update_flip_style(self,flip_string):
    self.flipstyle.direction = FlipStyle.HORZ # Default is Horizontal
    self.flip_code = 1

    if flip_string == 'vert':
        self.flip_code = 0
        rospy.loginfo('Flip direction is vertical')
        self.flipstyle.direction = FlipStyle.VERT
    else:
        rospy.loginfo('Flip direction is horizontal')

  def cbPubTimer(self,event):
    self.config_pub.publish(self.flipstyle)

  def callback(self,ros_data):
    np_arr = np.fromstring(ros_data.data, np.uint8)
    image_np = cv2.imdecode(np_arr, cv2.CV_LOAD_IMAGE_COLOR)
    image_np = cv2.flip(image_np, self.flip_code)

    #### Create CompressedIamge ####
    msg = CompressedImage()
    msg.header.stamp = rospy.Time.now()
    msg.format = "jpeg"
    msg.data = np.array(cv2.imencode('.jpg', image_np)[1]).tostring()
    self.image_pub.publish(msg)

def main(args):
  rospy.init_node('virtual_mirror_mrinal', anonymous=True)
  ip = image_processor()
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")

if __name__ == '__main__':
    main(sys.argv)