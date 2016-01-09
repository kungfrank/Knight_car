#!/usr/bin/env python
import sys
import rospy
from ground_projection.srv import EstimateHomography, GetGroundCoord
from sensor_msgs.msg import CameraInfo, Image
import numpy as np
import IPython

def call_service_estimate_homography(req):
  rospy.wait_for_service("/ground_projection/estimate_homography")

  try:
    service = rospy.ServiceProxy('/ground_projection/estimate_homography', EstimateHomography)
    resp = service(req)
    return resp.homography
  except rospy.ServiceException, e:
    print "Service call failed: %s" % e

if __name__ == "__main__":
  rospy.init_node("ex_estimate_homography")

  # get an image frame
  img = rospy.wait_for_message("/porsche911/rosberrypi_cam/image_rect", Image)
  print "got an image frame"

  H = call_service_estimate_homography(img)
  
  print H
  