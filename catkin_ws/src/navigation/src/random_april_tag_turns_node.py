#!/usr/bin/env python
import rospy
import numpy
from duckietown_msgs.msg import FSMState, AprilTags, BoolStamped
from std_msgs.msg import String, Int16 #Imports msg

class RandomAprilTagTurnsNode(object):
    def __init__(self):
        # Save the name of the node
        self.node_name = rospy.get_name()
        self.turn_type = -1

        rospy.loginfo("[%s] Initialzing." %(self.node_name))

        # Setup publishers
        # self.pub_topic_a = rospy.Publisher("~topic_a",String, queue_size=1)
        self.pub_turn_type = rospy.Publisher("~turn_type",Int16, queue_size=1, latch=True)

        # Setup subscribers
        # self.sub_topic_b = rospy.Subscriber("~topic_b", String, self.cbTopic)
        self.sub_topic_mode = rospy.Subscriber("~mode", FSMState, self.cbMode, queue_size=1)
        self.sub_topic_tag = rospy.Subscriber("~tag", AprilTags, self.cbTag, queue_size=1)
       

        # Read parameters
        self.pub_timestep = self.setupParameter("~pub_timestep",1.0)
        # Create a timer that calls the cbTimer function every 1.0 second
        #self.timer = rospy.Timer(rospy.Duration.from_sec(self.pub_timestep),self.cbTimer)

        rospy.loginfo("[%s] Initialzed." %(self.node_name))

        self.rate = rospy.Rate(30) # 10hz

    def cbMode(self, mode_msg):
        #print mode_msg
        self.fsm_mode = mode_msg.state
            
    def cbTag(self, tag_msgs):
        #loop through list of april tags
        for taginfo in tag_msgs.infos:
            print tag_msg
            if(taginfo.tag_type == taginfo.SIGN):
                aailableTurns = []
                #go through possible intersection types
                signType = taginfo.traffic_sign_type
                if(signType == NO_RIGHT_TURN or signType == LEFT_T_INTERSECT):
                    availableTurns = [1,2]
                elif (signType == NO_LEFT_TURN or signType == RIGHT_T_INTERSECT):
                    availableTurns = [0,1]
                elif (signType== FOUR_WAY):
                    availableTurns = [0,1,2]
                elif (signType == T_INTERSECTION):
                    availableTurns = [0,2]

                    #now randomly choose a possible direction
                if(len(availableTurns)>0):
                    chosenTurn = numpy.random.randint(1,len(availableTurns))
                    pub_turn_type.pub(chosenTurn)
                    rospy.loginfo("Turn type now: %i" %(self.turn_type))

    def setupParameter(self,param_name,default_value):
        value = rospy.get_param(param_name,default_value)
        rospy.set_param(param_name,value) #Write to parameter server for transparancy
        rospy.loginfo("[%s] %s = %s " %(self.node_name,param_name,value))
        return value

    def on_shutdown(self):
        rospy.loginfo("[%s] Shutting down." %(self.node_name))

if __name__ == '__main__':
    # Initialize the node with rospy
    rospy.init_node('random_april_tag_turns_node', anonymous=False)

    # Create the NodeName object
    node = RandomAprilTagTurnsNode()

    # Setup proper shutdown behavior 
    rospy.on_shutdown(node.on_shutdown)
    # Keep it spinning to keep the node alive
    rospy.spin()
