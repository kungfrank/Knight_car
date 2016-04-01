#!/usr/bin/env python
import rospy
from rgb_led import *
import sys
import time
from std_msgs.msg import Float32
import random

class LEDEmitterTest(object):
    def __init__(self):
        self.node_name = rospy.get_name()
        self.pub_state = rospy.Publisher("~change_to_state",Float32,queue_size=10)
        self.sub_state = rospy.Subscriber("~current_led_state",Float32, self.changeState)
        self.pub_timer = rospy.Timer(rospy.Duration.from_sec(5.0),self.cycleTimer)
        self.state_list = [1, 2, .5, .8] # From duckietown_lights.py

    def cycleTimer(self,event):
        state = random.choice(self.state_list)
        self.pub_state.publish(state)
        rospy.loginfo("Testing state " + str(state))


    def changeState(self,msg):
        rospy.loginfo("I see " + str(msg.data) + " as state")

if __name__ == '__main__':
    rospy.init_node('led_emitter',anonymous=False)
    node = LEDEmitterTest()
    rospy.spin()

