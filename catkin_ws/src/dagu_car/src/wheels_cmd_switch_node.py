#!/usr/bin/env python
import rospy
from duckietown_msgs.msg import WheelsCmdStamped, ControlMode

class WheelsCmdSwitchNode(object):
    def __init__(self):
        self.node_name = rospy.get_name()
        rospy.loginfo("[%s] Initializing " %(self.node_name))
        
        self.mode_msg = ControlMode()
        self.mode_msg.header.stamp = rospy.Time.now()
        self.mode_msg.mode = ControlMode.LANE_FOLLOWING

        self.cmd_dict = {}
        self.cmd_dict[ControlMode.LANE_FOLLOWING] = None
        self.cmd_dict[ControlMode.INTERSECTION_CONTROL] = None
        self.cmd_dict[ControlMode.COORDINATION_CONTROL] = None

        self.mode_name_dict = {}
        self.mode_name_dict[ControlMode.LANE_FOLLOWING] = "LANE_FOLLOWING"
        self.mode_name_dict[ControlMode.INTERSECTION_CONTROL] = "INTERSECTION_CONTROL"
        self.mode_name_dict[ControlMode.COORDINATION_CONTROL] = "COORDINATION_CONTROL"

        # Setup publishers
        self.pub_wheels_cmd = rospy.Publisher("~wheels_cmd", WheelsCmdStamped, queue_size=1)

        # Setup subscribers
        self.sub_mode = rospy.Subscriber("~mode", ControlMode, self.cbMode, queue_size=1)
        self.sub_lane = rospy.Subscriber("~wheels_cmd_lane", WheelsCmdStamped, self.cbWheelsCmd, queue_size=1, callback_args=ControlMode.LANE_FOLLOWING)
        self.sub_interestion = rospy.Subscriber("~wheels_cmd_intersection", WheelsCmdStamped, self.cbWheelsCmd, queue_size=1, callback_args=ControlMode.INTERSECTION_CONTROL)
        self.sub_coordination = rospy.Subscriber("~wheels_cmd_coordination", WheelsCmdStamped, self.cbWheelsCmd, queue_size=1, callback_args=ControlMode.COORDINATION_CONTROL)

    def pubWheelsCmd(self):
        cmd_msg = self.cmd_dict[mode]
        if cmd_msg is not None:
            self.pub_wheels_cmd.publish(cmd_msg)

    def cbMode(self,mode_msg):
        if not self.mode_msg.mode == mode_msg.mode:
            ropsy.loginfo("[%s] Switching to %s" %(self.node_name,self.mode_name_dict[mode_msg.mode]))
        self.mode_msg = mode_msg
        # Always publish a cmd when changing mode.
        self.pubWheelsCmd()

    def cbWheelsCmd(self,cmd_msg,cb_args):
        # Save the cmd_msg 
        self.cmd_dict[cb_args] = cmd_msg
        # Publish if the received cmd channel matches the current mode
        if cb_args == self.mode_msg.mode:
            self.pubWheelsCmd()

    # def setupParam(self,param_name,default_value):
    #     value = rospy.get_param(param_name,default_value)
    #     rospy.set_param(param_name,value) #Write to parameter server for transparancy
    #     rospy.loginfo("[%s] %s = %s " %(self.node_name,param_name,value))
    #     return value

    def on_shutdown(self):
        rospy.loginfo("[%s] Shutting down." %(self.node_name))

if __name__ == '__main__':
    # Initialize the node with rospy
    rospy.init_node('wheels_cmd_switch_node', anonymous=False)
    # Create the DaguCar object
    node = WheelsCmdSwitchNode()
    # Setup proper shutdown behavior 
    rospy.on_shutdown(node.on_shutdown)
    # Keep it spinning to keep the node alive
    rospy.spin()
