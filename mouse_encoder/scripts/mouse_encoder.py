#!/usr/bin/env python
# Converted from snippet on http://stackoverflow.com/questions/4855823/get-mouse-deltas-using-python-in-linux
import struct
import rospy
from geometry_msgs.msg import Point

class MouseEncoder(object):
    def __init__(self,dev_path):
        # Open the device
        self.dev = open(dev_path,"rb") #TODO: handle exception.
        rospy.loginfo("[MouseEncoder] Use %s" %(dev_path))
        self.pub_tick = rospy.Publisher('~tick',Point, queue_size=1)

    def getMouseTick(self,event):
        buf = self.dev.read(3);
        button = ord( buf[0] );
        bLeft = button & 0x1;
        bMiddle = ( button & 0x4 ) > 0;
        bRight = ( button & 0x2 ) > 0;
        x,y = struct.unpack( "bb", buf[1:] );
        # print ("L:%d, M: %d, R: %d, x: %d, y: %d\n" % (bLeft,bMiddle,bRight, x, y) );
        point = Point()
        point.x = x
        point.y = y
        self.pub_tick.publish(point)
        # rospy.loginfo("[x,y] = %s" %([x,y]))

if __name__ == '__main__':
    rospy.init_node('mouse_encoder',anonymous=False)
    # Set default parameters
    dev_path = "/dev/input/mice"
    # Read from paramter
    if rospy.has_param("~dev_path"):
        dev_path = rospy.get_param("~dev_path")

    mouse_odo = MouseEncoder(dev_path)
    
    moust_timer = rospy.Timer(rospy.Duration.from_sec(0.01),mouse_odo.getMouseTick)

    rospy.spin()