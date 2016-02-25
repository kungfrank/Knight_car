#!/usr/bin/env python
import rospy
from std_msgs.msg import String #Imports msg

# Define callback function
def callback(msg):
	rospy.loginfo("I heard: %s" %(msg.data))
	# TODO: Publish the received message using publisher

# Initialize the node with rospy
rospy.init_node('repeater_node')
# Create publisher
publisher = rospy.Publisher("~topic_out",String,queue_size=1)
# Create subscriber
subscriber = rospy.Subscriber("~topic_in", String, callback)

rospy.spin() #Keeps the script for exiting