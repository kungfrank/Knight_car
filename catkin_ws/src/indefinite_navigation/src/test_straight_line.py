#!/usr/bin/env python
import rospy
from intersection_control.util import HelloGoodbye #Imports module. Not limited to modules in this pkg. 
from duckietown_msgs.msg import FSMState

from std_msgs.msg import String #Imports msg
from std_msgs.msg import Bool #Imports msg
#from duckietown_msgs.msg import messages to command the wheels
from duckietown_msgs.msg import WheelsCmdStamped

class IndefNavigationNode(object):
    def __init__(self):
        # Save the name of the node
        self.node_name = rospy.get_name()
        
        rospy.loginfo("[%s] Initialzing." %(self.node_name))
	veh_name= rospy.get_param("veh")['duckiebot_visualizer']['veh_name']
	topic = veh_name + "/wheels_driver_node/wheels_cmd"

        # Setup publishers
        self.pub_wheels_cmd = rospy.Publisher(topic,WheelsCmdStamped, queue_size=1)
        # Create a timer that calls the cbTimer function every 1.0 second

        rospy.loginfo("[%s] Initialzed." %(self.node_name))

        self.rate = rospy.Rate(30) # 10hz

    def driveForward(self):
        #move forward
        forward_for_time = 3
        starting_time = rospy.Time.now()
        while((rospy.Time.now() - starting_time) < rospy.Duration(forward_for_time)):
            wheels_cmd_msg = WheelsCmdStamped()
            wheels_cmd_msg.header.stamp = rospy.Time.now()
            wheels_cmd_msg.vel_left = 0.4
            wheels_cmd_msg.vel_right = 0.4
            self.pub_wheels_cmd.publish(wheels_cmd_msg)    
            rospy.loginfo("Moving?.")
            self.rate.sleep()
    

if __name__ == '__main__':
    # Initialize the node with rospy
    rospy.init_node('indef_navigation_node', anonymous=False)

    # Create the NodeName object
    node = IndefNavigationNode()
    raw_input("drive forward?")
    node.driveForward()

