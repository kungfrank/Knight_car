#!/usr/bin/env python
import rospy
from pkg_name.modulename import ModuleName #Imports module. Not limited to modules in this pkg. 
from std_msgs.msg import String #Imports msg

class NodeName(object):
    def __init__(self):
        # Setup publishers
        self.pub_topic = rospy.Publisher("~publish_topic_name",String, queue_size=1)

        # Setup subscribers
        self.sub_topic = rospy.Subscriber("~subscribe_topic_name", String, self.cbTopic)

        # Create a timer that calls the cbTimer function every 1.0 second
        self.timer = rospy.Timer(rospy.Duration.from_sec(1.0),self.cbTimer)
        rospy.loginfo("[NodeName] Initialzed.")

    def cbTopic(self,msg):
        rospy.loginfo("[NodeName] I heard " + msg.data)
        # Republishes the msg
        self.pub_topic.publish(msg)

    def cbTimer(self,event):
        module = ModuleName()
        # Simulate hearing something
        msg = String()
        msg.data = module.method_name("world")
        self.cbTopic(msg)

    def on_shutdown(self):
        rospy.loginfo("[NodeName] Shutting down.")

if __name__ == '__main__':
    # Initialize the node with rospy
    rospy.init_node('node_name', anonymous=False)

    # Create the NodeName object
    node = NodeName()

    # Setup proper shutdown behavior 
    rospy.on_shutdown(node.on_shutdown)
    
    # Keep it spinning to keep the node alive
    rospy.spin()