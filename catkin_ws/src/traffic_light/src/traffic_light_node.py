#!/usr/bin/env python
import rospy
from rgb_led import *
import sys
import time
from std_msgs.msg import Float32, Int8
from rgb_led import RGB_LED


class TrafficLight(object):
    def __init__(self):
        self.led = RGB_LED()
        self.node_name = rospy.get_name()
        self.cycle = None

        self.traffic_light_list = self.setupParameter("~traffic_light_list",[0,2,3,1]); #order of lights
        self.greenlight_duration = self.setupParameter("~greenlight_duration",3) #in seconds
        self.allred_duration = self.setupParameter("~allred_duration",7) #in seconds
        self.redlight_freq  = self.setupParameter("~redlight_freq",2) # in Hz
        self.greenlight_freq = self.setupParameter("~greenlight_freq",4) # in Hz
        
        self.redlight_t = 1.0/self.redlight_freq
        self.greenlight_t = 1.0/self.greenlight_freq
        self.green_on=False
        self.green_i = 0;
        self.green = self.traffic_light_list[self.green_i]
        self.redlightlist = self.traffic_light_list[:self.green_i] + self.traffic_light_list[(self.green_i+1):];
        self.traffic_light_state = {0:False,1:False,2:False,3:False} #All LEDs are off

        self.traffic_cycle = rospy.Timer(rospy.Duration((self.allred_duration+self.greenlight_duration)),self.switchGreen)
        self.redLED_cycle = rospy.Timer(rospy.Duration(self.redlight_t),self.freqred)
        self.greenLED_cycle = rospy.Timer(rospy.Duration(self.greenlight_t),self.freqgreen)

    def switchGreen(self,event):
        self.green_i = (self.green_i+1)%4 #Move to next light in list
        self.green = self.traffic_light_list[self.green_i]
        self.green_on=True
        self.redlightlist = self.traffic_light_list[:self.green_i] + self.traffic_light_list[(self.green_i+1):];
        rospy.sleep(self.greenlight_duration) #Keep the green light on
        self.green_on = False #Turn off the green light
        self.redlightlist = self.traffic_light_list[0:]

    def freqred(self,event):
        for light in self.redlightlist:
            if self.traffic_light_state[light]==True:
                self.led.setRGB(light,[0,0,0])
                self.traffic_light_state[light]=False
            else:
                self.led.setRGB(light,[1,0,0])
                self.traffic_light_state[light]=True

    def freqgreen(self,event):
        if self.green_on==False: #Exit if lights should all be red
            return
        if self.traffic_light_state[self.green]==True:
            self.led.setRGB(self.green,[0,0,0])
            self.traffic_light_state[self.green]=False
        else:
            self.led.setRGB(self.green,[0,1,0])
            self.traffic_light_state[self.green]=True

    def setupParameter(self,param_name,default_value):
        value = rospy.get_param(param_name,default_value)
        rospy.set_param(param_name,value) #Write to parameter server for transparancy
        rospy.loginfo("[%s] %s = %s " %(self.node_name,param_name,value))
        return value

    def changePattern(self,msg): #NOT BEING USED
        self.pattern_index = msg.data

    def changeState(self,msg): #NOT BEING USED
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
        self.pub_state.publish(self.cycle)

if __name__ == '__main__':
    rospy.init_node('traffic_light',anonymous=False)
    node = TrafficLight()
    rospy.spin()

