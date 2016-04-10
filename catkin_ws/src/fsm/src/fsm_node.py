#!/usr/bin/env python
import rospy
from duckietown_msgs.msg import FSMState, BoolStamped
from duckietown_msgs.srv import SetFSMState, SetFSMStateRequest, SetFSMStateResponse

class FSMNode(object):
    def __init__(self):
        self.node_name = rospy.get_name()

        # Build transition dictionary
        self.states_dict = rospy.get_param("~states")
        # validate type, just in case
        assert isinstance(self.states_dict, dict), type(self.states_dict)

        # Setup initial state
        self.state_msg = FSMState()
        self.state_msg.state = rospy.get_param("~initial_state")
        self.state_msg.header.stamp = rospy.Time.now()
        # Setup publisher and publish initial state
        self.pub_state = rospy.Publisher("~mode", FSMState, queue_size=1, latch=True)
    
        # Provide service
        self.srv_state = rospy.Service("~set_state", SetFSMState, self.cbSrvSetState)

        # Construct publishers
        self.pub_dict = dict()
        nodes = rospy.get_param("~nodes")
        for node_name, topic_name in nodes.items():
            p = rospy.Publisher(topic_name, BoolStamped, queue_size=1, latch=True)
            self.pub_dict[node_name] = p

        # Construct subscribers
        param_events_dict = rospy.get_param("~events")
        self.sub_list = []
        self.event_names = []
        for event_name, topic_name in param_events_dict.items():
            s = rospy.Subscriber("%s" % (topic_name), BoolStamped, self.cbEvent,
                                  callback_args=event_name)
            self.sub_list.append(s)

        # Publish initial state
        self.publish()

    def _getNextState(self, state_name, event_name):
        state_dict = self.states_dict.get(state_name)
        if state_dict is None:
            rospy.logwarn("[%s] %s not defined. Treat as terminal. "
                          % (self.node_name, state_name))
            return None
        else:
            if "transitions" in state_dict.keys():
                next_state = state_dict["transitions"].get(event_name)
            else:
                next_state = None
            return next_state
                

    def _getActiveNodes(self,state_name):
        state_dict = self.states_dict.get(state_name)
        if state_dict is None:
            rospy.logwarn("[%s] %s not defined. Treat as terminal. " %
                          (self.node_name, state_name))
            return None
        else:
            active_nodes = state_dict.get("active_nodes")
            if active_nodes is None:
                rospy.logwarn("[%s] No active nodes defined for %s. Deactive all nodes."
                              % (self.node_name, state_name))
                active_nodes = []
            return active_nodes

    def publish(self):
        self.publishBools()
        self.publishState()

    def cbSrvSetState(self,req):
        self.state_msg.header.stamp = rospy.Time.now()
        self.state_msg.state = req.state
        self.publish()
        return SetFSMStateResponse()

    def publishState(self):
        self.pub_state.publish(self.state_msg)
        rospy.loginfo("[%s] FSMState: %s" % (self.node_name, self.state_msg.state))

    def publishBools(self):
        msg = BoolStamped()
        msg.header.stamp = self.state_msg.header.stamp
        active_nodes = self._getActiveNodes(self.state_msg.state)
        if active_nodes is None:
            active_nodes = []

        for node_name, node_pub in self.pub_dict.items():
            msg.data = False
            node_state = "OFF"
            if node_name in active_nodes:
                msg.data = True
                node_state = "ON"
            node_pub.publish(msg)
            rospy.loginfo("[%s] Node %s set to %s." %
                          (self.node_name, node_name, node_state))

    def cbEvent(self,msg,event_name):
        if (msg.data):
            # Update timestamp
            rospy.loginfo("[%s] Event: %s" % (self.node_name, event_name))
            self.state_msg.header.stamp = msg.header.stamp
            next_state = self._getNextState(self.state_msg.state,event_name)
            if next_state is not None:
                # Has a defined transition
                self.state_msg.state = next_state
                self.publish()

    def on_shutdown(self):
        rospy.loginfo("[%s] Shutting down." % self.node_name)

if __name__ == '__main__':
    # Initialize the node with rospy
    rospy.init_node('fsm_node', anonymous=False)

    # Create the NodeName object
    node = FSMNode()

    # Setup proper shutdown behavior 
    rospy.on_shutdown(node.on_shutdown)
    # Keep it spinning to keep the node alive
    rospy.spin()
