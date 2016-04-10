#!/usr/bin/env python
import rospy
#from duckietown_msgs.msg import WheelsCmdStamped, ControlMode
from duckietown_msgs.msg import Twist2DStamped, FSMState

class CarCmdSwitchNode(object):
    def __init__(self):
        self.node_name = rospy.get_name()
        mode_table = rospy.get_param("~mode_table")
        self.mode_table = dict()
        for key in mode_table:
            self.mode_table[int(key)] = mode_table[key] 

        self.fsm_state = FSMState()
        # publisher
        self.pub_cmd = rospy.Publisher("~cmd",Twist2DStamped,queue_size=1)
        # subscribe to mode
        self.sub_mode = rospy.Subscriber("~mode",FSMState,self.cbFSMState,queue_size=1)
        # Read subscription id and topic names
        sub_table = rospy.get_param("~sub_name")
        self.sub_dict = dict()
        for key in sub_table.keys():
            # TODO: confirm queue_size effectiveness
            self.sub_dict[int(key)] = rospy.Subscriber(sub_table[key],Twist2DStamped,self.cbTwist2DStamped,callback_args=int(key),queue_size=1)

    def cbTwist2DStamped(self,msg,cb_arg):
        # Only publish if it's from the source specifed by the current mode
        if cb_arg == self.mode_table[self.fsm_state.state]:
            self.pub_cmd.publish(msg)

    def cbFSMState(self,msg):
        if msg.state not in self.mode_table.keys():
            rospy.logerr("[%s] Mode not recognized. Using previous mode." %(self.node_name))
        else:
            self.fsm_state = msg

    def on_shutdown(self):
        rospy.loginfo("[%s] Shutting down." %(self.node_name))

if __name__ == '__main__':
    # Initialize the node with rospy
    rospy.init_node('car_cmd_switch_node', anonymous=False)
    # Create the DaguCar object
    node = CarCmdSwitchNode()
    # Setup proper shutdown behavior 
    rospy.on_shutdown(node.on_shutdown)
    # Keep it spinning to keep the node alive
    rospy.spin()
