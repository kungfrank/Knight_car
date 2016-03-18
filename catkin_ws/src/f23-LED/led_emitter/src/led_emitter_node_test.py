#!/usr/bin/env python
import rospy
from rgb_led import *
import sys
import time
from std_msgs.msg import String
import random

class LEDEmitterTest(object):
    def __init__(self):
        self.node_name = rospy.get_name()
        self.pub_state = rospy.Publisher("~change_state",String,queue_size=1)
        self.sub_state = rospy.Subscriber("~led_state",String, self.changeState)
        self.pub_timer = rospy.Timer(rospy.Duration.from_sec(5.0),self.cycleTimer)
        self.state_list = ['blinking1', 'blinking2', 'blinking3', 'mar12special'] # From duckietown_lights.py

    def cycleTimer(self,event):
        state = random.choice(self.state_list)
        self.pub_state.publish(state)
        rospy.loginfo("Testing state " + state)


    def changeState(self,msg):
        rospy.loginfo("I see " + str(msg.data) + " as state")

if __name__ == '__main__':
    rospy.init_node('led_emitter',anonymous=False)
    node = LEDEmitterTest()
    rospy.spin()

