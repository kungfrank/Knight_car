#!/usr/bin/env python
import sys
import rospy
from ground_projection.srv import EstimateHomography, GetGroundCoord
from duckietown_msgs.msg import Pixel
from geometry_msgs.msg import Point
import numpy as np
import IPython

def call_service_get_ground_coordinate(req, veh):
  rospy.wait_for_service(veh + "/ground_projection/get_ground_coordinate")

  try:
    get_ground_coord = rospy.ServiceProxy(veh + '/ground_projection/get_ground_coordinate', GetGroundCoord)
    resp = get_ground_coord(req)
    return resp.gp
  except rospy.ServiceException, e:
    print "Service call failed: %s" % e

if __name__ == "__main__":
  if len(sys.argv) != 2:
    print "usage: " + sys.argv[0] + " vehicle name"
    sys.exit()

  veh = "/" + sys.argv[1]

  rospy.init_node("ex_get_ground_coordinate")

  uv = Pixel()
  uv.u = 320
  uv.v = 300

  gp = call_service_get_ground_coordinate(uv, veh)

  print "ground coordinate: (%f, %f, %f)" % (gp.x, gp.y, gp.z)
