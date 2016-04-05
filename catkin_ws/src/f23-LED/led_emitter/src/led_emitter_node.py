#!/usr/bin/env python
import rospy
from rgb_led import *
import sys
import time
from std_msgs.msg import Float32, Int8
from rgb_led import RGB_LED


class LEDEmitter(object):
    def __init__(self):
        self.led = RGB_LED()
        self.node_name = rospy.get_name()
        self.pub_state = rospy.Publisher("~current_led_state",Float32,queue_size=1)
        self.sub_cycle_state = rospy.Subscriber("~change_light_frequency",Float32, self.changeState)
        self.sub_pattern_state = rospy.Subscriber("~change_color_pattern",Int8, self.changePattern)
        self.cycle = None
        self.pattern_index = 0
        self.is_on = False
        self.color_pattern_list = [([0,0,0], [0,0,0], [1,1,1], [0,0,0], [0,0,0]),
                                    ([0,0,0], [0,0,0], [0,0,1], [0,0,0], [0,0,0]),
                                    ([0,0,0], [0,0,0], [1,0,0], [0,0,0], [0,0,0])]
        self.cycle_timer = rospy.Timer(rospy.Duration.from_sec(.1),self.cycleTimer)
        # take current time
        # self.t0 = <time>

    def cycleTimer(self,event):
        if self.is_on:
            for items in ([0,1,2,3,4]):
                self.led.setRGB(items, [0,0,0])
                self.is_on = False
        else:
            for items in ([0,1,2,3,4]):
                self.led.setRGB(items, self.color_pattern_list[self.pattern_index][items])
                self.is_on = True

    def changePattern(self,msg):
        self.pattern_index = msg.data

    def changeState(self,msg):
        if msg.data == self.cycle:
            self.cycle = msg.data
            # It is the same nothing changed
        else:     
            try:
                self.cycle = msg.data
                self.cycle_timer.shutdown()
                #below, convert to hz
                self.cycle_timer = rospy.Timer(rospy.Duration.from_sec(1.0/(2.0*self.cycle)),self.cycleTimer)
            except ValueError as e:
                self.cycle = None
        self.pub_state.publish(float(self.cycle))

if __name__ == '__main__':
    rospy.init_node('led_emitter',anonymous=False)
    node = LEDEmitter()
    rospy.spin()

