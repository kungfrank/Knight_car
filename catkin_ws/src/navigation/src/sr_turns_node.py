#!/usr/bin/env python
import rospy
import numpy
from duckietown_msgs.msg import FSMState, AprilTags, BoolStamped
from std_msgs.msg import String, Int16 #Imports msg

class SRTurnsNode(object):
    def __init__(self):
        # Save the name of the node
        self.node_name = rospy.get_name()
        self.turn_type = -1

        rospy.loginfo("[%s] Initialzing." %(self.node_name))

        # Setup publishers
        self.pub_turn_type = rospy.Publisher("~turn_type",Int16, queue_size=1, latch=True)

        # Setup subscribers
        self.sub_topic_mode = rospy.Subscriber("~mode", FSMState, self.cbMode, queue_size=1)
       
        rospy.loginfo("[%s] Initialzed." %(self.node_name))

        self.rate = rospy.Rate(30) # 10hz

    def cbMode(self, mode_msg):
        #print mode_msg
        self.fsm_mode = mode_msg.state
        if(self.fsm_mode == "INTERSECTION_CONTROL"):
            # return only straight and right turn
            availableTurns = [1,2]
            #now randomly choose a possible direction
            if(len(availableTurns)>0):
                randomIndex = numpy.random.randint(len(availableTurns))
                chosenTurn = availableTurns[randomIndex]
                self.turn_type = chosenTurn
                self.pub_turn_type.publish(self.turn_type)
                rospy.loginfo("[%s] possible turns %s." %(self.node_name,availableTurns))
                rospy.loginfo("[%s] Turn type now: %i" %(self.node_name,self.turn_type))
        else:
            self.turn_type = -1
            self.pub_turn_type.publish(self.turn_type)
            rospy.loginfo("[%s] Turn type: %i" %(self.node_name, self.turn_type))

    def on_shutdown(self):
        rospy.loginfo("[%s] Shutting down." %(self.node_name))

if __name__ == '__main__':
    # Initialize the node with rospy
    rospy.init_node('sr_turns_node', anonymous=False)

    # Create the NodeName object
    node = SRTurnsNode()

    # Setup proper shutdown behavior 
    rospy.on_shutdown(node.on_shutdown)
    # Keep it spinning to keep the node alive
    rospy.spin()
