#!/usr/bin/env python
import rospy
from duckietown_msgs.msg import FSMState, BoolStamped

class FSMEnforcerNode(object):
    def __init__(self):
        self.node_name = rospy.get_name()

        # Read states defintion and tables
        self.state_name_dict = dict()
        name_state_dict = rospy.get_param("~states")
        for state_name,state_id in name_state_dict.items():
            self.state_name_dict[state_id] = state_name

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
        state_name = self.state_name_dict.get(state_msg.state)
        if state_name is None:
            rospy.logerr("[%s] Unhandled state. FSMState.state = %s." %(self.node_name,state_msg.state))
            return
        
        switch_table = self.tables.get(state_name)
        if switch_table is None:
            rospy.logerr("[%s] No table defined for %s. Treat as empty." %(self.node_name, state_name))
            # Construct empty switch table
            switch_table = dict()
            switch_table["on"] = list()
            switch_table["off"] = list()

        # Publisher BoolStamped according to the table
        msg = BoolStamped()
        msg.header.stamp = state_msg.header.stamp
        for switch_name in switch_table["turn_off"]:
            msg.data = False
            self.publish(switch_name,msg)
        for switch_name in switch_table["turn_on"]:
            msg.data = True
            self.publish(switch_name,msg)

    def publish(self,switch_name,msg):
        if switch_name in self.pub_dict.keys():
            self.pub_dict[switch_name].publish(msg)
            switch_state = "OFF"
            if msg.data:
                switch_state = "ON"
            rospy.loginfo("[%s] Trun %s %s"%(self.node_name,switch_name,switch_state))
        else:
            rospy.logerr("[%s] switch name %s not defined." %(self.node_name, switch_name))

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
