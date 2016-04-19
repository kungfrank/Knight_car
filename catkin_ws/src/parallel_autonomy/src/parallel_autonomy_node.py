#!/usr/bin/env python
import rospy
import numpy
from duckietown_msgs.msg import FSMState, AprilTags, BoolStamped
from std_msgs.msg import String, Int16 #Imports msg
from sensor_msgs.msg import Joy
from rgb_led import RGB_LED

class ParallelAutonomyNode(object):
    def __init__(self):
        # Save the name of the node
        self.node_name = rospy.get_name()

        self.led = RGB_LED()
        self.turn_LEFT = 0
        self.turn_RIGHT = 1
        self.turn_STRAIGHT = 2
        self.turn_NONE = 3

        self.blink = False
	rospy.loginfo("hello3")
        self.availableTurns = []
        self.turn_direction = self.turn_NONE

        rospy.loginfo("[%s] Initialzing." %(self.node_name))

        # Setup publishers
        self.pub_turn_type = rospy.Publisher("~turn_type",Int16, queue_size=1, latch=True)

        # Setup subscribers
        self.sub_topic_mode = rospy.Subscriber("~mode", FSMState, self.cbMode, queue_size=1)
        self.sub_topic_tag = rospy.Subscriber("~tag", AprilTags, self.cbTag, queue_size=1)
        self.sub_joy = rospy.Subscriber("~joy", Joy, self.cbJoy, queue_size=1)

        self.cycle_timer = rospy.Timer(rospy.Duration.from_sec(.5), self.blinkTimer)
       
        rospy.loginfo("[%s] Initialzed." %(self.node_name))

        self.rate = rospy.Rate(30) # 10hz

    def blinkTimer(self,event):
        self.led.setRGB(4, [.2, .2, .2])
        self.led.setRGB(2, [.2, .2, .2])
        self.led.setRGB(3, [.3, 0, 1])
        self.led.setRGB(1, [.3, 0, 0])
        self.led.setRGB(0, [.2, .2, .2])
        if self.turn_direction == self.turn_LEFT:
            if self.blink:
                self.led.setRGB(1, [1,0, 0])
                self.led.setRGB(0, [1, 1, 1])
                self.blink = False
            else:
                self.led.setRGB(1, [.3, 0, 0])
                self.led.setRGB(0, [.2, .2, .2])
                self.blink = True
        if self.turn_direction == self.turn_RIGHT:
            if self.blink:
                self.led.setRGB(3, [1,0, 0])
                self.led.setRGB(4, [1, 1, 1])
                self.blink = False
            else:
                self.led.setRGB(3, [.3, 0, 0])
                self.led.setRGB(4, [.2, .2, .2])
                self.blink = True


    def cbJoy(self,msg):
	rospy.loginfo("hello2")
        if msg.buttons[2] and self.turn_LEFT in self.availableTurns: # or self.joy.axes[3] > 0.2:
            if self.turn_direction == self.turn_LEFT:
                self.turn_direction = self.turn_STRAIGHT
            else:
                self.turn_direction = self.turn_LEFT
        elif msg.buttons[1] and self.turn_RIGHT in self.availableTurns: #or self.joy.axes[3] < -0.2:
            if self.turn_direction == self.turn_RIGHT:
                self.turn_direction = self.turn_STRAIGHT
            else:
                self.turn_direction = self.turn_RIGHT
        if self.turn_direction == self.turn_STRAIGHT and self.turn_STRAIGHT not in self.availableTurns and len(self.availableTurns)>0:
            self.turn_direction = self.turn_NONE



    def cbTag(self, tag_msgs):
        if(self.fsm_mode == "WAITING_FOR_TURN_DIRECTION"):
            #loop through list of april tags
            for taginfo in tag_msgs.infos:
                print taginfo
                rospy.loginfo("[%s] taginfo." %(taginfo))
                if(taginfo.tag_type == taginfo.SIGN):
                    self.availableTurns = []
                    #go through possible intersection types
                    signType = taginfo.traffic_sign_type
                    if(signType == taginfo.NO_RIGHT_TURN or signType == taginfo.LEFT_T_INTERSECT):
                        self.availableTurns = [self.turn_STRAIGHT,self.turn_LEFT]
                    elif (signType == taginfo.NO_LEFT_TURN or signType == taginfo.RIGHT_T_INTERSECT):
                        self.availableTurns = [self.turn_RIGHT,self.turn_STRAIGHT]
                    elif (signType== taginfo.FOUR_WAY):
                        self.availableTurns = [self.turn_RIGHT,self.turn_STRAIGHT,self.turn_LEFT]
                    elif (signType == taginfo.T_INTERSECTION):
                        self.availableTurns = [self.turn_RIGHT,self.turn_LEFT]

    def cbMode(self, mode_msg):
        #print mode_msg
        self.fsm_mode = mode_msg.state

	rospy.loginfo(self.fsm_mode)
	print "hello"

        if(self.fsm_mode == "WAITING_FOR_TURN_DIRECTION"):
	    rospy.loginfo("hello")
            self.availableTurns = [0,1,2]
            self.turn_direction = self.turn_RIGHT
            rospy.sleep(2)
            while self.turn_direction is self.turn_NONE:
                pass
            self.pub_turn_type.publish(self.turn_direction) # make sure mapping to turn_type is ok
            rospy.loginfo("[%s] Turn type: %i" %(self.node_name, self.turn_direction))
            
    

    def setupParameter(self,param_name,default_value):
        value = rospy.get_param(param_name,default_value)
        rospy.set_param(param_name,value) #Write to parameter server for transparancy
        rospy.loginfo("[%s] %s = %s " %(self.node_name,param_name,value))
        return value

    def on_shutdown(self):
        rospy.loginfo("[%s] Shutting down." %(self.node_name))

if __name__ == '__main__':
    # Initialize the node with rospy
    rospy.init_node('parallel_autonomy_node', anonymous=False)
    rospy.loginfo("hello1st")
    print "hello1st"
    # Create the NodeName object
    node = ParallelAutonomyNode()

    # Setup proper shutdown behavior 
    rospy.on_shutdown(node.on_shutdown)
    # Keep it spinning to keep the node alive
    rospy.spin()

