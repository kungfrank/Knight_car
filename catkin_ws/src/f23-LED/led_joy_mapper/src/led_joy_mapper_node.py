#!/usr/bin/env python
import rospy
import numpy as np
from sensor_msgs.msg import Joy
from std_msgs.msg import Float32, Int8
import time


# Button List index of joy.buttons array:
# a = 0, b=1, x=2. y=3, lb=4, rb=5, back = 6, start =7, logitek = 8, left joy = 9, right joy = 10
class LEDJoyMapper(object):
    def __init__(self):
        self.node_name = rospy.get_name()
        rospy.loginfo("[%s] Initializing " %(self.node_name))
        
        self.joy = None
        self.last_pub_msg = None
        self.last_pub_time = rospy.Time.now()

        # Publications
        self.pub_lights = rospy.Publisher("~change_light_frequency", Float32, queue_size=1)
        self.pub_color = rospy.Publisher("~change_color_pattern", Int8, queue_size=1)
        # Subscriptions
        self.sub_joy_ = rospy.Subscriber("joy", Joy, self.cbJoy, queue_size=1)
        self.broadcast_patterns = [3.5, 4.1, 5.0]
        self.color_index = [0,1,2]

    def cbJoy(self, joy_msg):
        self.joy = joy_msg
        self.publishControl()

    def publishControl(self):
        if self.joy.buttons[0] == 1:
            self.pub_lights.publish(self.broadcast_patterns[0]) # 'a' is pressed
            rospy.loginfo("Publishing pattern A")
        elif self.joy.buttons[1] == 1:
            self.pub_lights.publish(self.broadcast_patterns[1]) # 'b' is pressed
            rospy.loginfo("Publishing pattern B")
        elif self.joy.buttons[3] == 1:
            rospy.loginfo("Publishing pattern C")
            self.pub_lights.publish(self.broadcast_patterns[2]) # 'c' is pressed
        elif self.joy.buttons[4] == 1:
            rospy.loginfo("Publishing color pattern 0")
            self.pub_color.publish(self.color_index[0]) # lb is pressed
        elif self.joy.buttons[5] == 1 :
            rospy.loginfo("Publishing color pattern 1")
            self.pub_color.publish(self.color_index[1]) # rb is pressed
        elif self.joy.buttons[8] == 1:
            rospy.loginfo("Publishing color pattern 2")
            self.pub_color.publish(self.color_index[2]) # logitek button is pressed

if __name__ == "__main__":
    rospy.init_node("led_joy_mapper",anonymous=False)
    led_joy_mapper = LEDJoyMapper()
    rospy.spin()
