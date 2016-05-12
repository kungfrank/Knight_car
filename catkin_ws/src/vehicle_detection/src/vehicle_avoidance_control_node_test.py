#!/usr/bin/env python

import rospy
from std_msgs.msg import String, Bool
from duckietown_msgs.msg import WheelsCmdStamped, VehiclePose, FSMState

class VehicleAvoidanceControlNodeTest:

    def __init__(self):
        self.node_name = "Vehicle Avoidance Control Node Test"

        self.wheels_cmd_sub = rospy.Subscriber("~vehicle_avoidance_control",WheelsCmdStamped, self.cbCmd, queue_size = 1)
        self.vehicle_detected_sub = rospy.Subscriber("~vehicle_detected",Bool, self.cbDetected, queue_size=1)
        self.in_lane_pub = rospy.Publisher("~in_lane",Bool, queue_size=1)
        self.fsm_mode_sub = rospy.Subscriber("~mode",FSMState,self.cbMode,queue_size=1)
        self.wheel_cmd_switch_sub = rospy.Subscriber("~switch_commands",WheelsCmdStamped,self.cbWheelSwitch, queue_size=1)
        self.pose_pub = rospy.Publisher("~vehicle_pose",VehiclePose, queue_size = 1)

        self.rho = 0.1

        rospy.loginfo("Initialization of [%s] completed" % (self.node_name))

        rospy.Timer(rospy.Duration.from_sec(1.0), self.pubPose)
        rospy.Timer(rospy.Duration.from_sec(1.0), self.pubInLane)

    def pubPose(self,args=None):
        pose_msg_out = VehiclePose()

        pose_msg_out.rho.data = self.rho
        pose_msg_out.theta.data = 0.0
        pose_msg_out.psi.data = 0.0

        self.rho = self.rho + 0.1

        self.pose_pub.publish(pose_msg_out)

    def pubInLane(self,args=None):
        self.in_lane_pub.publish(True)

    def cbWheelSwitch(self,switch_msg):
        rospy.loginfo('SWITCH OUTPUT : (left = %.2f, right = %.2f)' % 
            (switch_msg.vel_left, switch_msg.vel_right))

    def cbCmd(self, cmd_msg):
        rospy.loginfo('Command received : (left = %.2f, right = %.2f)' %
                    (cmd_msg.vel_left, cmd_msg.vel_right))

    def cbDetected(self,detected_msg):
        rospy.loginfo('Vehicle detected? : %r' %
                    (detected_msg))

    def cbMode(self, mode_msg):
        rospy.loginfo('Mode : %d' % (mode_msg.state))

if __name__ == '__main__':
    rospy.init_node('vehicle_avoidance_control_node_test', anonymous=False)
    vehicle_avoidance_control_node_test = VehicleAvoidanceControlNodeTest()
    rospy.spin()