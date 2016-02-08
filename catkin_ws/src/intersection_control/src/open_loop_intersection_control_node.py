#!/usr/bin/env python
import rospy
from pkg_name.util import HelloGoodbye #Imports module. Not limited to modules in this pkg. 
from std_msgs.msg import String #Imports msg
#from duckietown_msgs.msg import messages to command the wheels
from duckietown_msgs.msg import WheelsCmdStamped
class Talker(object):
    def __init__(self):
        # Save the name of the node
        self.node_name = rospy.get_name()
        
        rospy.loginfo("[%s] Initialzing." %(self.node_name))

        # Setup publishers
        self.pub_topic_a = rospy.Publisher("~topic_a",String, queue_size=1)
        self.pub_wheels_cmd = rospy.Publisher("/kitt/wheels_driver_node/wheels_cmd",WheelsCmdStamped, queue_size=1)
        # Setup subscriber
        self.sub_topic_b = rospy.Subscriber("~topic_b", String, self.cbTopic)
        # Read parameters
        self.pub_timestep = self.setupParameter("~pub_timestep",1.0)
        # Create a timer that calls the cbTimer function every 1.0 second
        self.timer = rospy.Timer(rospy.Duration.from_sec(self.pub_timestep),self.cbTimer)

        rospy.loginfo("[%s] Initialzed." %(self.node_name))

        rate = rospy.Rate(30) # 10hz
        #move forward
        forward_for_time_leave = 2.0
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
            rate.sleep()
        #turn right
        starting_time = rospy.Time.now()
        while((rospy.Time.now() - starting_time) < rospy.Duration(turn_for_time)):
            wheels_cmd_msg = WheelsCmdStamped()
            wheels_cmd_msg.header.stamp = rospy.Time.now()
            wheels_cmd_msg.vel_left = 0.25
            wheels_cmd_msg.vel_right = -0.25
            self.pub_wheels_cmd.publish(wheels_cmd_msg)    
            rospy.loginfo("Moving?.")
            rate.sleep()
   
        #move forward
        starting_time = rospy.Time.now()
        while((rospy.Time.now() - starting_time) < rospy.Duration(forward_for_time_enter)):
            wheels_cmd_msg = WheelsCmdStamped()
            wheels_cmd_msg.header.stamp = rospy.Time.now()
            wheels_cmd_msg.vel_left = 0.4
            wheels_cmd_msg.vel_right = 0.4
            self.pub_wheels_cmd.publish(wheels_cmd_msg)    
            rospy.loginfo("Moving?.")
            rate.sleep()
   
    def setupParameter(self,param_name,default_value):
        value = rospy.get_param(param_name,default_value)
        rospy.set_param(param_name,value) #Write to parameter server for transparancy
        rospy.loginfo("[%s] %s = %s " %(self.node_name,param_name,value))
        return value

    def cbTopic(self,msg):
        rospy.loginfo("[%s] %s" %(self.node_name,msg.data))

    def cbTimer(self,event):
        singer = HelloGoodbye()
        # Simulate hearing something
        msg = String()
        msg.data = singer.sing("duckietown")
        self.pub_topic_a.publish(msg)
#        wheels_cmd_msg = WheelsCmdStamped()
#        wheels_cmd_msg.vel_left = 0.1
#        wheels_cmd_msg.vel_right = 0.1
#        self.pub_wheels_cmd.publish(wheels_cmd_msg)

    def on_shutdown(self):
        rospy.loginfo("[%s] Shutting down." %(self.node_name))

if __name__ == '__main__':
    # Initialize the node with rospy
    rospy.init_node('talker', anonymous=False)

    # Create the NodeName object
    node = Talker()

    # Setup proper shutdown behavior 
    rospy.on_shutdown(node.on_shutdown)
    # Keep it spinning to keep the node alive
    rospy.spin()
