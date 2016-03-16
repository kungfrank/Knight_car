#!/usr/bin/env python
import rospy
from std_msgs.msg import String #Imports msg
# Initialize the node with rospy
rospy.init_node('publisher_node')
# Create publisher
publisher = rospy.Publisher("~topic",String,queue_size=1)
# Define Timer callback
def callback(event):
    msg = String()
    msg.data = "Hello your_name!"
    publisher.publish(msg)
# Create timer 
rospy.Timer(rospy.Duration.from_sec(1.0),callback)
# spin to keep the script for exiting
rospy.spin() 
