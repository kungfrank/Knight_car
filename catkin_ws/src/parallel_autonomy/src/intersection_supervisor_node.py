#!/usr/bin/env python
import rospy
import numpy
from duckietown_msgs.msg import FSMState, AprilTagsWithInfos, BoolStamped
from std_msgs.msg import String, Int16  # Imports msg
from sensor_msgs.msg import Joy
from rgb_led import RGB_LED


class IntersectionSupervisorNode(object):
    def __init__(self):
        # Save the name of the node
        self.node_name = rospy.get_name()

        self.led = RGB_LED()
        self.turn_LEFT = 0
        self.turn_RIGHT = 2
        self.turn_STRAIGHT = 1
        self.turn_NONE = 3

        self.blink = False
        self.availableTurns = []
        self.turn_direction = self.turn_NONE
        self.joy_forward = 0
        self.timeout = 5  # seconds
        rospy.loginfo("[%s] Initializing." % self.node_name)

        # Setup publishers
        self.pub_turn_type = rospy.Publisher("~turn_type", Int16, queue_size=1, latch=True)
	self.pub_done = rospy.Publisher("~intersection_done",BoolStamped,queue_size=1)

        # Setup subscribers
        self.sub_topic_mode = rospy.Subscriber("~mode", FSMState, self.cbMode, queue_size=1)
        self.sub_topic_tag = rospy.Subscriber("~tag", AprilTagsWithInfos, self.cbTag, queue_size=1)
        self.sub_joy = rospy.Subscriber("~joy", Joy, self.cbJoy, queue_size=1)

        self.cycle_timer = rospy.Timer(rospy.Duration.from_sec(.5), self.blinkTimer)

        rospy.loginfo("[%s] Initialized." % self.node_name)

        self.rate = rospy.Rate(30)  # 10hz

        # led setup
        self.led.setRGB(4, [.2, .2, .2])
        self.led.setRGB(2, [.2, .2, .2])
        self.led.setRGB(3, [.3, 0, 0])
        self.led.setRGB(1, [.3, 0, 0])
        self.led.setRGB(0, [.2, .2, .2])

    def blinkTimer(self, event):
        if self.turn_direction == self.turn_LEFT:
            if self.blink:
                self.led.setRGB(1, [1, 0, 0])
                self.led.setRGB(0, [1, 1, 1])
                self.blink = False
            else:
                self.led.setRGB(1, [.3, 0, 0])
                self.led.setRGB(0, [.2, .2, .2])
                self.blink = True
        if self.turn_direction == self.turn_RIGHT:
            if self.blink:
                self.led.setRGB(3, [1, 0, 0])
                self.led.setRGB(4, [1, 1, 1])
                self.blink = False
            else:
                self.led.setRGB(3, [.3, 0, 0])
                self.led.setRGB(4, [.2, .2, .2])
                self.blink = True
        if self.turn_direction == self.turn_STRAIGHT:
            # lights back to normal
            self.led.setRGB(0, [.2, .2, .2])
            self.led.setRGB(4, [.2, .2, .2])
            self.led.setRGB(3, [.3, 0, 0])
            self.led.setRGB(1, [.3, 0, 0])

    def cbJoy(self, msg):
        self.joy_forward = msg.axes[1]
        if msg.buttons[2] and self.turn_LEFT in self.availableTurns:  # or self.joy.axes[3] > 0.2:
            if self.turn_direction == self.turn_LEFT:
                self.turn_direction = self.turn_STRAIGHT
            else:
                self.turn_direction = self.turn_LEFT
        elif msg.buttons[1] and self.turn_RIGHT in self.availableTurns:  # or self.joy.axes[3] < -0.2:
            if self.turn_direction == self.turn_RIGHT:
                self.turn_direction = self.turn_STRAIGHT
            else:
                self.turn_direction = self.turn_RIGHT
        if self.turn_direction == self.turn_STRAIGHT and self.turn_STRAIGHT not in self.availableTurns and len(self.availableTurns) > 0:
            self.turn_direction = self.turn_NONE
        if self.turn_direction == self.turn_NONE and self.turn_STRAIGHT in self.availableTurns:
            self.turn_direction = self.turn_STRAIGHT

    def cbTag(self, tag_msgs): # this and random_april_tag_turns are duplicated should be turned into a library
        if self.fsm_mode == "INTERSECTION_CONTROL":
            # loop through list of april tags
            for taginfo in tag_msgs.infos:
                print taginfo
                rospy.loginfo("[%s] tag info." % taginfo)
                if taginfo.tag_type == taginfo.SIGN:
                    self.availableTurns = []
                    # go through possible intersection types
                    signType = taginfo.traffic_sign_type
                    if signType == taginfo.NO_RIGHT_TURN or signType == taginfo.LEFT_T_INTERSECT:
                        self.availableTurns = [self.turn_STRAIGHT, self.turn_LEFT]
                    elif signType == taginfo.NO_LEFT_TURN or signType == taginfo.RIGHT_T_INTERSECT:
                        self.availableTurns = [self.turn_RIGHT, self.turn_STRAIGHT]
                    elif signType == taginfo.FOUR_WAY:
                        self.availableTurns = [self.turn_RIGHT, self.turn_STRAIGHT, self.turn_LEFT]
                    elif signType == taginfo.T_INTERSECTION:
                        self.availableTurns = [self.turn_RIGHT, self.turn_LEFT]

    def cbMode(self, mode_msg):
        self.fsm_mode = mode_msg.state

        if (self.fsm_mode == "INTERSECTION_CONTROL"):
            # self.availableTurns = [0,1,2]  # just to test without april tags, set to [] for actual
            # brake lights
            self.led.setRGB(3, [1, 0, 0])
            self.led.setRGB(1, [1, 0, 0])
            # force to stop for 2 seconds
            rospy.sleep(2)
            # default to straight if nothing pressed
            self.turn_direction = self.turn_NONE
            # if no straight turn avaliable wait until another is choosen
            # publish once user presses forward on joystick
            time_max = rospy.get_time() + self.timeout
            while (self.turn_direction == self.turn_NONE or not self.joy_forward > 0) and not (rospy.get_time() > time_max and self.availableTurns ==[]):
                pass
            # turn off brake lights
            self.led.setRGB(1, [.3, 0, 0])
	    self.led.setRGB(3, [.3, 0, 0])
            if self.availableTurns == []:
		#if timeout and no available turns, just leave instersection control 
		done = BoolStamped()
	        done.header.stamp = rospy.Time.now()
        	done.data = True
		self.pub_done.publish(done)
	    else:
	        self.pub_turn_type.publish(self.turn_direction)
            rospy.loginfo("[%s] Turn type: %i" % (self.node_name, self.turn_direction))
        if self.fsm_mode != "INTERSECTION_CONTROL":
            # on exit intersection control, stop blinking
            self.availableTurns = []
            self.turn_direction = self.turn_NONE
            # led setup
            self.led.setRGB(4, [.2, .2, .2])
            self.led.setRGB(2, [.2, .2, .2])
            self.led.setRGB(3, [.3, 0, 0])
            self.led.setRGB(1, [.3, 0, 0])
            self.led.setRGB(0, [.2, .2, .2])

    def setupParameter(self, param_name, default_value):
        value = rospy.get_param(param_name, default_value)
        rospy.set_param(param_name, value)  # Write to parameter server for transparancy
        rospy.loginfo("[%s] %s = %s " % (self.node_name, param_name, value))
        return value

    def on_shutdown(self):
        rospy.loginfo("[%s] Shutting down." % (self.node_name))


if __name__ == '__main__':
    # Initialize the node with rospy
    rospy.init_node('intersection_supervisor_node', anonymous=False)
    # Create the NodeName object
    node = IntersectionSupervisorNode()

    # Setup proper shutdown behavior 
    rospy.on_shutdown(node.on_shutdown)
    # Keep it spinning to keep the node alive
    rospy.spin()
