#!/usr/bin/env python
import rospy
from duckietown_msgs.msg import FSMState, BoolStamped

class FSMNode(object):
    def __init__(self):
        self.node_name = rospy.get_name()

        self.state_msg = FSMState()
        # TODO: set default state
        self.pub_state = rospy.Publisher("~state",FSMState,queue_size=1)
        
        # Build dict from FSMState.mode to name of states and transition table for each state
        self.name_state_dict = rospy.get_param("~states")
        self.state_name_dict = dict()
        self.state_trans_dict = dict()
        for state_name,state_id in self.name_state_dict.items():
            self.state_name_dict[state_id] = state_name
            # TODO: check if param exsit, complain and shutdown if not.
            self.state_trans_dict[state_id] = rospy.get_param("~%s"%(state_name))

        # Construct subscribers
        param_events_dict = rospy.get_param("~events")
        self.sub_list = list()
        self.event_id_name_dict = dict()
        for event_name,event_id in param_events_dict.items():
            self.sub_list.append(rospy.Subscriber("~%s"%(event_name), BoolStamped, self.cbEvent, callback_args=event_id))
            self.event_id_name_dict[event_id] = event_name

    def cbEvent(self,msg,event_id):
        if (msg.data):
            self.state_msg.header.stamp = msg.header.stamp
            trans_dict = self.state_trans_dict.get(self.state_msg.state)
            if trans_dict is None:
                # TODO ERROR state not handled
                return
            event_name = self.event_id_name_dict[event_id]
            next_state_name = trans_dict.get(event_name)
            if next_state_name is not None:
                next_state_id = self.name_state_dict.get(next_state_name)
                rospy.loginfo("[%s] FSMState: %s" %(self.node_name, next_state_name))
                # if not next_state_id == self.state_msg.state:
                self.state_msg.state = next_state_id
                self.pub_state.publish(self.state_msg)

    def on_shutdown(self):
        rospy.loginfo("[%s] Shutting down." %(self.node_name))

if __name__ == '__main__':
    # Initialize the node with rospy
    rospy.init_node('fsm_node', anonymous=False)

    # Create the NodeName object
    node = FSMNode()

    # Setup proper shutdown behavior 
    rospy.on_shutdown(node.on_shutdown)
    # Keep it spinning to keep the node alive
    rospy.spin()
