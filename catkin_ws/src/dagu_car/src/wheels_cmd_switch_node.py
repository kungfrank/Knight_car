#!/usr/bin/env python
import rospy
#from duckietown_msgs.msg import WheelsCmdStamped, ControlMode
from duckietown_msgs.msg import WheelsCmdStamped, FSMState

class WheelsCmdSwitchNode(object):
    def __init__(self):
        self.node_name = rospy.get_name()
        rospy.loginfo("[%s] Initializing " %(self.node_name))
        
#        self.mode_msg = ControlMode()
        self.mode_msg = FSMState()
        #self.mode_msg.header.stamp = rospy.Time.now()
#        self.mode_msg.mode = ControlMode.LANE_FOLLOWING
        self.mode_msg.state = FSMState.LANE_FOLLOWING
        self.cmd_dict = {}
#        self.cmd_dict[ControlMode.LANE_FOLLOWING] = None
#        self.cmd_dict[ControlMode.INTERSECTION_CONTROL] = None
#        self.cmd_dict[ControlMode.COORDINATION_CONTROL] = None
        self.cmd_dict[FSMState.LANE_FOLLOWING] = None
        self.cmd_dict[FSMState.INTERSECTION_CONTROL] = None
        self.cmd_dict[FSMState.COORDINATION] = None
        self.cmd_dict[FSMState.VEHICLE_AVOIDANCE] = None

        self.mode_name_dict = {}
#        self.mode_name_dict[ControlMode.LANE_FOLLOWING] = "LANE_FOLLOWING"
 #       self.mode_name_dict[ControlMode.INTERSECTION_CONTROL] = "INTERSECTION_CONTROL"
  #      self.mode_name_dict[ControlMode.COORDINATION_CONTROL] = "COORDINATION_CONTROL"
        self.mode_name_dict[FSMState.LANE_FOLLOWING] = "LANE_FOLLOWING"
        self.mode_name_dict[FSMState.INTERSECTION_CONTROL] = "INTERSECTION_CONTROL"
        self.mode_name_dict[FSMState.COORDINATION] = "COORDINATION_CONTROL"
        self.mode_name_dict[FSMState.VEHICLE_AVOIDANCE] = "VEHICLE_AVOIDANCE"

        # Setup publishers
        self.pub_wheels_cmd = rospy.Publisher("~wheels_cmd", WheelsCmdStamped, queue_size=1)

        # Setup subscribers
        self.sub_mode = rospy.Subscriber("~mode", FSMState, self.cbMode, queue_size=1)
        self.sub_lane = rospy.Subscriber("~wheels_cmd_lane", WheelsCmdStamped, self.cbWheelsCmd, queue_size=1, callback_args=FSMState.LANE_FOLLOWING)
        self.sub_interestion = rospy.Subscriber("~wheels_cmd_intersection", WheelsCmdStamped, self.cbWheelsCmd, queue_size=1, callback_args=FSMState.INTERSECTION_CONTROL)
        self.sub_coordination = rospy.Subscriber("~wheels_cmd_coordination", WheelsCmdStamped, self.cbWheelsCmd, queue_size=1, callback_args=FSMState.COORDINATION)
        self.sub_vehicle_avoidance = rospy.Subscriber("~wheels_cmd_avoidance", WheelsCmdStamped, self.cbWheelsCmd, queue_size=1, callback_args=FSMState.VEHICLE_AVOIDANCE)

    def pubWheelsCmd(self):
        cmd_msg = self.cmd_dict[self.mode_msg.state]
        if cmd_msg is None:
            cmd_msg = WheelsCmdStamped()
            cmd_msg.vel_left=0.0
            cmd_msg.vel_right=0.0
            cmd_msg.header.stamp = rospy.Time.now()
        self.pub_wheels_cmd.publish(cmd_msg)

    def cbMode(self,mode_msg):
        if not self.mode_msg.state == mode_msg.state:
            rospy.loginfo("[%s] Switching to %s" %(self.node_name,self.mode_name_dict[mode_msg.state]))
        self.mode_msg = mode_msg
        # Always publish a cmd when changing mode.
        self.pubWheelsCmd()

    def cbWheelsCmd(self,cmd_msg,cb_args):
        # Save the cmd_msg 
        self.cmd_dict[cb_args] = cmd_msg
        # Publish if the received cmd channel matches the current mode
        if cb_args == self.mode_msg.state:
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
