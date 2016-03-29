#!/usr/bin/env python
import rospy
from duckietown_msgs.msg import FSMState, BoolStamped

class FSMEnforcerNode(object):
    def __init__(self):
        self.node_name = rospy.get_name()

        # Read tables
        self.tables = rospy.get_param("~tables")

        # Setup publishers
        self.pub_dict = dict()
        switches = rospy.get_param("~switches")
        for switch_name, topic_name in switches.items():
            self.pub_dict[switch_name] = rospy.Publisher(topic_name, BoolStamped, queue_size=1, latch=True)
        
        # Setup subscriber
        self.sub_state = rospy.Subscriber("~state", FSMState, self.cbState)

    def cbState(self, state_msg):
        rospy.loginfo("[%s] State: %s" %(self.node_name, state_msg.state))        
        msg = BoolStamped()
        msg.header.stamp = state_msg.header.stamp
        on_switch_list = self.tables.get(state_msg.state)
        
        if on_switch_list is None:
            rospy.logwarn("[%s] Unhandled state %s. All switch set to OFF." %(self.node_name,state_msg.state))
            on_switch_list = []

        for switch_name, pub_switch in self.pub_dict.items():
            msg.data = False
            switch_state = "OFF"
            if switch_name in on_switch_list:
                msg.data = True
                switch_state = "ON"
            pub_switch.publish(msg)
            rospy.loginfo("[%s] %s set to %s" %(self.node_name, switch_name, switch_state))

    def on_shutdown(self):
        rospy.loginfo("[%s] Shutting down." %(self.node_name))

if __name__ == '__main__':
    # Initialize the node with rospy
    rospy.init_node('fsm_node', anonymous=False)

    # Create the NodeName object
    node = FSMEnforcerNode()

    # Setup proper shutdown behavior 
    rospy.on_shutdown(node.on_shutdown)
    # Keep it spinning to keep the node alive
    rospy.spin()
