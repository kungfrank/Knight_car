#!/usr/bin/env python
import rospy
from rgb_led import *
import sys
import time
from std_msgs.msg import String

class LEDEmitter(object):
    def __init__(self):
        self.node_name = rospy.get_name()
        self.pub_state = rospy.Publisher("~led_state",String,queue_size=1)
        self.sub_state = rospy.Subscriber("~change_state",String, self.changeState)
        self.prior_pattern = ""

    def changeState(self,msg):
        if msg.data == self.prior_pattern:
            pattern = msg.data
            # It is the same nothing changed
        else:     
            try:
                pattern = msg.data
                cycle_LEDs_named(pattern)
            except ValueError as e:
                pattern = "[UNKNOWN PATTERN]"
                rospy.loginfo("Unknown pattern")
        self.pub_state.publish(pattern)

if __name__ == '__main__':
    rospy.init_node('led_emitter',anonymous=False)
    node = LEDEmitter()
    rospy.spin()

