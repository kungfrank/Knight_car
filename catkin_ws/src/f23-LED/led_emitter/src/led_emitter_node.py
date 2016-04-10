#!/usr/bin/env python
import rospy
from rgb_led import *
import sys
import time
from std_msgs.msg import Float32, Int8, String
from rgb_led import RGB_LED


class LEDEmitter(object):
    def __init__(self):
        self.led = RGB_LED()
        self.node_name = rospy.get_name()
        self.pub_state = rospy.Publisher("~current_led_state",Float32,queue_size=1)
        self.sub_pattern = rospy.Subscriber("~change_color_pattern", String, self.changePattern)
        self.cycle = None
        
        self.is_on = False

        self.changePattern_('CAR_SIGNAL_A')

        self.cycle_timer = rospy.Timer(rospy.Duration.from_sec(.1), self.cycleTimer)

        # take current time
        # self.t0 = <time>

    def cycleTimer(self,event):
        if self.is_on:
            for i in range(5):
                self.led.setRGB(i, [0, 0, 0])
                self.is_on = False
        else:
            for i in range(5):
                self.led.setRGB(i, self.pattern[i])
                self.is_on = True

    def changePattern(self, msg):
        self.changePattern_(msg.data)

    def changePattern_(self, pattern_name):
        rospy.loginfo('changePattern(%r)' % pattern_name)

        colors = {
            'green': [0,1,0],
            'red':   [1,0,0],
            'blue':  [0,0,1],
            'white': [0,0,0],
            'yellow': [1,1,0],
            'purple': [1,0,1], 
            'cyan': [0,1,1],
            'black': [0,0,0],
        }

        scale = 0.5

        for _, c in colors.items():
            for i in range(3):
                c[i] = c[i]  * scale

        f1 = 2.8
        f2 = 4.1
        f3 = 5.0 

        if pattern_name  in ['off', 'CAR_SIGNAL_A', 'CAR_SIGNAL_B',   'CAR_SIGNAL_C']:
            m = {
              'CAR_SIGNAL_A': ('green', f1),
              'CAR_SIGNAL_B': ('purple', f2),
              'CAR_SIGNAL_C': ('yellow', f3),
              'off': ('black', f3),
            }
            color, self.cycle = m[pattern_name]

            self.pattern = [[0,0,0]] * 5
            self.pattern[2] = colors[color]
            self.pattern_off = [[0,0,0]]*5

        elif pattern_name in ['traffic_light_go', 'traffic_light_stop']:
            m = {
            'traffic_light_go':  ('green', f1),
            'traffic_light_stop': ('red', f3),
            }
            color, self.cycle = m[pattern_name]
            self.pattern = [colors[color]] * 5
            self.pattern_off = [[0,0,0]]*5



    def changeState(self,msg):
        if msg.data == self.cycle:
            self.cycle = msg.data
            # It is the same nothing changed
        else:     
            try:
                self.cycle = msg.data
                self.cycle_timer.shutdown()
                #below, convert to hz
                d = 1.0/(2.0*self.cycle)
                self.cycle_timer = rospy.Timer(rospy.Duration.from_sec(d), self.cycleTimer)
            except ValueError as e:
                self.cycle = None
        self.pub_state.publish(float(self.cycle))

if __name__ == '__main__':
    rospy.init_node('led_emitter',anonymous=False)
    node = LEDEmitter()
    rospy.spin()

