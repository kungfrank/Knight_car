#!/usr/bin/env python
import rospy
from duckietown_msgs.msg import Twist2DStamped, BoolStamped
class SimpleStopControllerNode:
    def __init__(self):
        self.name = 'simple_stop_controller_node'
        rospy.loginfo('[%s] started', self.name)
        self.sub_close = rospy.Subscriber("~too_close", BoolStamped, self.cbBool, queue_size=1)
        self.pub_car_cmd = rospy.Publisher("~car_cmd",Twist2DStamped,queue_size=1)

    def cbBool(self, bool_msg):
        stop = Twist2DStamped()
        stop.header = bool_msg.header
        stop.v = 0.0
        stop.omega = 0.0
        self.pub_car_cmd.publish(stop)

if __name__=="__main__":
    rospy.init_node('simple_stop_controller_node')
    node = SimpleStopControllerNode()
    rospy.spin()
