#!/usr/bin/env python
import rospy
from duckietown_msgs.msg import FSMState, BoolStamped
from duckietown_msgs.srv import SetFSMState, SetFSMStateRequest, SetFSMStateResponse

class FSMNode(object):
    def __init__(self):
        self.node_name = rospy.get_name()

        # Build transition dictionray
        self.transition_dict = rospy.get_param("~transitions")
        rospy.loginfo("[%s] Defined states: %s" %(self.node_name,self.transition_dict.keys()))

        # Setup initial state
        self.state_msg = FSMState()
        self.state_msg.state = rospy.get_param("~initial_state")
        self.state_msg.header.stamp = rospy.Time.now()
        # Setup publisher and publish initial state
        self.pub_state = rospy.Publisher("~state",FSMState,queue_size=1,latch=True)
        self.publishState()
    
        # Provide service
        self.srv_state = rospy.Service("~set_state",SetFSMState,self.cbSrvSetState)

        # Construct subscribers
        param_events_dict = rospy.get_param("~events")
        self.sub_list = list()
        self.event_names = list()
        for event_name, topic_name in param_events_dict.items():
            self.sub_list.append(rospy.Subscriber("%s"%(topic_name), BoolStamped, self.cbEvent, callback_args=event_name))

    def cbSrvSetState(self,req):
        self.state_msg.header.stamp = rospy.Time.now()
        self.state_msg.state = req.state
        self.publishState()
        return SetFSMStateResponse()

    def publishState(self):
        self.pub_state.publish(self.state_msg)
        rospy.loginfo("[%s] FSMState: %s" %(self.node_name, self.state_msg.state))

    def cbEvent(self,msg,event_name):
        if (msg.data):
            # Update timestamp
            rospy.loginfo("[%s] Event: %s" %(self.node_name,event_name))
            self.state_msg.header.stamp = msg.header.stamp
            # Load transitions of the current state
            trans_dict = self.transition_dict.get(self.state_msg.state)
            # rospy.loginfo("[%s] Transiniton %s" %(self.node_name,trans_dict))
            if trans_dict is None:
                # current state is not defined  
                rospy.logwarn("[%s] %s is terminal state."%(self.node_name,self.state_msg.state))
                # rospy.signal_shutdown("Undefined state.")
            else:
                next_state = trans_dict.get(event_name)
                if next_state is not None:
                    # Has transition for the event
                    self.state_msg.state = next_state
                    self.publishState()

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
