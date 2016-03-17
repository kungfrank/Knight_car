#!/usr/bin/env python
import rospy
from duckietown_msgs.msg import FSMState, BoolStamped, WheelsCmdStamped
from std_msgs.msg import String, Int16 #Imports msg

class OpenLoopIntersectionNode(object):
    def __init__(self):
        # Save the name of the node
        self.node_name = rospy.get_name()
        self.turn_type = 0

        rospy.loginfo("[%s] Initialzing." %(self.node_name))

        # Setup publishers
        # self.pub_topic_a = rospy.Publisher("~topic_a",String, queue_size=1)
        self.pub_wheels_cmd = rospy.Publisher("~wheels_cmd",WheelsCmdStamped, queue_size=1)
        self.pub_wheels_done = rospy.Publisher("~intersection_done",BoolStamped, queue_size=1)

        # Setup subscribers
        # self.sub_topic_b = rospy.Subscriber("~topic_b", String, self.cbTopic)
        self.sub_topic_mode = rospy.Subscriber("~mode", FSMState, self.cbMode, queue_size=1)
        self.sub_topic_turn_type = rospy.Subscriber("~turn_type", Int16, self.cbTurnType, queue_size=1)

        # Read parameters
        self.pub_timestep = self.setupParameter("~pub_timestep",1.0)
        # Create a timer that calls the cbTimer function every 1.0 second
        #self.timer = rospy.Timer(rospy.Duration.from_sec(self.pub_timestep),self.cbTimer)

        rospy.loginfo("[%s] Initialzed." %(self.node_name))

        self.rate = rospy.Rate(30) # 10hz

    def cbMode(self, mode_msg):
        print mode_msg
        if(mode_msg.state == mode_msg.INTERSECTION_CONTROL):
            if(self.turn_type==0):
                self.turnRight()
            elif (self.turn_type==1):
                self.turnStraight()
            elif(self.turn_type==2):
                self.turnLeft()

            rospy.loginfo("Turn type now: %i" %(self.turn_type))
        else:
            # If not in intersection control mode anymore, pubisher intersection_done False.
            self.setIntersectionDone(False)

    def cbTurnType(self, turn_msg):
        print turn_msg
        self.turn_type = turn_msg.data
        rospy.loginfo("Turn type now: %i" %(self.turn_type))

    def setIntersectionDone(self,state):
        boolstamped = BoolStamped()
        boolstamped.header.stamp = rospy.Time.now()
        boolstamped.data = state
        self.pub_wheels_done.publish(boolstamped)

    def turnRight(self):
        #move forward
        forward_for_time_leave = 1.2
        turn_for_time = 0.6
        forward_for_time_enter = 2.0
        
        starting_time = rospy.Time.now()
        while((rospy.Time.now() - starting_time) < rospy.Duration(forward_for_time_leave)):
            wheels_cmd_msg = WheelsCmdStamped()
            wheels_cmd_msg.header.stamp = rospy.Time.now()
            wheels_cmd_msg.vel_left = 0.4
            wheels_cmd_msg.vel_right = 0.4
            self.pub_wheels_cmd.publish(wheels_cmd_msg)    
            rospy.loginfo("Moving?.")
            self.rate.sleep()
        #turn right
        starting_time = rospy.Time.now()
        while((rospy.Time.now() - starting_time) < rospy.Duration(turn_for_time)):
            wheels_cmd_msg = WheelsCmdStamped()
            wheels_cmd_msg.header.stamp = rospy.Time.now()
            wheels_cmd_msg.vel_left = 0.25
            wheels_cmd_msg.vel_right = -0.25
            self.pub_wheels_cmd.publish(wheels_cmd_msg)    
            rospy.loginfo("Moving?.")
            self.rate.sleep()
   
        #coordination with lane controller means part way through announce finished turn
        self.setIntersectionDone(True)

        #move forward
        starting_time = rospy.Time.now()
        while((rospy.Time.now() - starting_time) < rospy.Duration(forward_for_time_enter)):
            wheels_cmd_msg = WheelsCmdStamped()
            wheels_cmd_msg.header.stamp = rospy.Time.now()
            wheels_cmd_msg.vel_left = 0.4
            wheels_cmd_msg.vel_right = 0.4
            self.pub_wheels_cmd.publish(wheels_cmd_msg)    
            rospy.loginfo("Moving?.")
            self.rate.sleep()

 
    def turnLeft(self):
        #move forward
        forward_for_time_leave = 3.0
        turn_for_time = 0.7
        forward_for_time_enter = 2.0
        
        starting_time = rospy.Time.now()
        while((rospy.Time.now() - starting_time) < rospy.Duration(forward_for_time_leave)):
            wheels_cmd_msg = WheelsCmdStamped()
            wheels_cmd_msg.header.stamp = rospy.Time.now()
            wheels_cmd_msg.vel_left = 0.4
            wheels_cmd_msg.vel_right = 0.4
            self.pub_wheels_cmd.publish(wheels_cmd_msg)    
            rospy.loginfo("Moving?.")
            self.rate.sleep()
        #turn left
        starting_time = rospy.Time.now()
        while((rospy.Time.now() - starting_time) < rospy.Duration(turn_for_time)):
            wheels_cmd_msg = WheelsCmdStamped()
            wheels_cmd_msg.header.stamp = rospy.Time.now()
            wheels_cmd_msg.vel_left = -0.25
            wheels_cmd_msg.vel_right = 0.25
            self.pub_wheels_cmd.publish(wheels_cmd_msg)    
            rospy.loginfo("Moving?.")
            self.rate.sleep()
   
        #coordination with lane controller means part way through announce finished turn
        self.setIntersectionDone(True)

        #move forward
        starting_time = rospy.Time.now()
        while((rospy.Time.now() - starting_time) < rospy.Duration(forward_for_time_enter)):
            wheels_cmd_msg = WheelsCmdStamped()
            wheels_cmd_msg.header.stamp = rospy.Time.now()
            wheels_cmd_msg.vel_left = 0.4
            wheels_cmd_msg.vel_right = 0.4
            self.pub_wheels_cmd.publish(wheels_cmd_msg)    
            rospy.loginfo("Moving?.")
            self.rate.sleep()
   
    def turnStraight(self):
        #move forward
        forward_for_time_leave = 3.0
        #turn_for_time = 0.7
        forward_for_time_enter = 3.0
        
        starting_time = rospy.Time.now()
        while((rospy.Time.now() - starting_time) < rospy.Duration(forward_for_time_leave)):
            wheels_cmd_msg = WheelsCmdStamped()
            wheels_cmd_msg.header.stamp = rospy.Time.now()
            wheels_cmd_msg.vel_left = 0.4
            wheels_cmd_msg.vel_right = 0.4
            self.pub_wheels_cmd.publish(wheels_cmd_msg)    
            rospy.loginfo("Moving?.")
            self.rate.sleep()
   
        #coordination with lane controller means part way through announce finished turn
        self.setIntersectionDone(True)

        #move forward
        starting_time = rospy.Time.now()
        while((rospy.Time.now() - starting_time) < rospy.Duration(forward_for_time_enter)):
            wheels_cmd_msg = WheelsCmdStamped()
            wheels_cmd_msg.header.stamp = rospy.Time.now()
            wheels_cmd_msg.vel_left = 0.4
            wheels_cmd_msg.vel_right = 0.4
            self.pub_wheels_cmd.publish(wheels_cmd_msg)    
            rospy.loginfo("Moving?.")
            self.rate.sleep()

    def setupParameter(self,param_name,default_value):
        value = rospy.get_param(param_name,default_value)
        rospy.set_param(param_name,value) #Write to parameter server for transparancy
        rospy.loginfo("[%s] %s = %s " %(self.node_name,param_name,value))
        return value

    def on_shutdown(self):
        rospy.loginfo("[%s] Shutting down." %(self.node_name))

if __name__ == '__main__':
    # Initialize the node with rospy
    rospy.init_node('open_loop_intersection_node', anonymous=False)

    # Create the NodeName object
    node = OpenLoopIntersectionNode()

    # Setup proper shutdown behavior 
    rospy.on_shutdown(node.on_shutdown)
    # Keep it spinning to keep the node alive
    rospy.spin()
