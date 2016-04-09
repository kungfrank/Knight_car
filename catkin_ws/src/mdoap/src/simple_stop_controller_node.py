#!/usr/bin/env python
import rospy
from duckietown_msgs.msg import WheelsCmdStamped, BoolStamped
class SimpleStopControllerNode:
    def __init__(self):
        self.name = 'simple_stop_controller_node'
        rospy.loginfo('[%s] started', self.name)
        self.sub_close = rospy.Subscriber("~too_close", BoolStamped, self.cbBool, queue_size=1)
        self.pub_wheels_cmd = rospy.Publisher("~control",WheelsCmdStamped,queue_size=1)

    def cbBool(self, bool_msg):
        stop = WheelsCmdStamped()
        stop.header = bool_msg.header
        stop.vel_left = 0.0
        stop.vel_right = 0.0
        self.pub_wheels_cmd.publish(stop)

if __name__=="__main__":
    rospy.init_node('simple_stop_controller_node')
    node = SimpleStopControllerNode()
    rospy.spin()