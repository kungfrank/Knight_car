#!/usr/bin/env python
import rospy
from duckietown_msgs.msg import FSMState, BoolStamped

class FSMNode(object):
    def __init__(self):
        self.node_name = rospy.get_name()
        self.state_msg = FSMState()
        # TODO: set default state
        self.pub_state = rospy.Publisher("~state",FSMState,queue_size=1,latch=True)
        
        # Build dict from FSMState.mode to name of states and transition table for each state
        self.name_state_dict = rospy.get_param("~states")
        self.state_name_dict = dict()
        self.state_trans_dict = dict()
        for state_name,state_id in self.name_state_dict.items():
            self.state_name_dict[state_id] = state_name
            if rospy.has_param("~transitions/%s"%(state_name)):
                self.state_trans_dict[state_id] = rospy.get_param("~transitions/%s"%(state_name))
                rospy.loginfo("[%s] state: %s transitions loaded."%(self.node_name,state_name))
            else:
                self.state_trans_dict[state_id] = dict()
                rospy.loginfo("[%s] state: %s has no transitions defined."%(self.node_name,state_name))

        # Construct subscribers
        param_events_dict = rospy.get_param("~events")
        self.sub_list = list()
        self.event_names = list()
        for event_name, topic_name in param_events_dict.items():
            self.sub_list.append(rospy.Subscriber("%s"%(topic_name), BoolStamped, self.cbEvent, callback_args=event_name))

    def cbEvent(self,msg,event_name):
        if (msg.data):
            # Update timestamp
            self.state_msg.header.stamp = msg.header.stamp
            # Load transitions of the current state
            trans_dict = self.state_trans_dict.get(self.state_msg.state)
            if trans_dict is None:
                # State not defined
                rospy.logerr("[%s] Transition into undefined state %s."%(self.node_name,next_state_name))
                rospy.signal_shutdown("Undefined state.")

            next_state_name = trans_dict.get(event_name)
            if next_state_name is not None:
                # Has transition for the event
                if next_state_name not in self.name_state_dict.keys():
                    # Transition into undefined state
                    rospy.logerr("[%s] Transition into undefined state %s."%(self.node_name,next_state_name))
                    rospy.signal_shutdown("Undefined state.")
                else:
                    # Transitoin into next state according to event
                    next_state_id = self.name_state_dict.get(next_state_name)
                    rospy.loginfo("[%s] FSMState: %s" %(self.node_name, next_state_name))
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
