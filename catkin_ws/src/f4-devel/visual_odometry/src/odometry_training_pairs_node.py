#!/usr/bin/env python
import rospy
from duckietown_msgs.msg import StopLineReading, LanePose, WheelsCmdStamped, Vsample
from std_msgs.msg import Float32
from geometry_msgs.msg import Point
import math


class OdometryTrainingPairsNode(object):
    def __init__(self):
        self.node_name = "Odometry Training Pairs"

        ## state vars
        self.in_lane = False
        self.current_wheels_cmd = WheelsCmdStamped()
        self.old_stop_line_msg = StopLineReading()

        ## publishers and subscribers
        self.sub_stop_lin_reading = rospy.Subscriber("~stop_line_reading", StopLineReading, self.stopLineCB)
        self.sub_lane_pose = rospy.Subscriber("~lane_pose", LanePose, self.lanePoseCB)
        self.sub_wheels_cmd_executed = rospy.Subscriber("~wheels_cmd_executed", WheelsCmdStamped, self.wheelsCmdCB)
        self.pub_theta_dot_sample = rospy.Publisher("~theta_dot_sample", Vsample, queue_size=1)
        rospy.loginfo('[odometry_training_pairs_node] Initiated')

    def stopLineCB(self, stop_line_msg):
        if self.in_lane and self.current_wheels_cmd and stop_line_msg.header.stamp.secs - self.old_stop_line_msg.header.stamp.secs <= 1.0 and self.old_stop_line_msg:
            theta_dot_sample_msg = Vsample()
            theta_dot_sample_msg.d_L = self.current_wheels_cmd.vel_left
            theta_dot_sample_msg.d_R = self.current_wheels_cmd.vel_right
            theta_dot_sample_msg.dt = stop_line_msg.header.stamp.secs + stop_line_msg.header.stamp.nsecs/1e9 - self.old_stop_line_msg.header.stamp.secs - self.old_stop_line_msg.header.stamp.nsecs/1e9
            theta_dot_sample_msg.theta_angle_pose_delta = 0
            theta_dot_sample_msg.x_axis_pose_delta = 0
            theta_dot_sample_msg.y_axis_pose_delta = self.old_stop_line_msg.stop_line_point.y - stop_line_msg.stop_line_point.y
            self.pub_theta_dot_sample.publish(theta_dot_sample_msg)
            self.old_stop_line_msg = stop_line_msg

    def lanePoseCB(self, lane_pose_msg):
        self.in_lane = lane_pose_msg.in_lane

    def wheelsCmdCB(self, wheels_cmd_msg):
        self.current_wheels_cmd = wheels_cmd_msg

    def processLanePose(self, lane_pose_msg):
        self.lane_pose = lane_pose_msg

    def onShutdown(self):
        rospy.loginfo("[odometry_training_pairs_node] Shutdown.")

if __name__ == '__main__':
    rospy.init_node('odometry_training_pairs',anonymous=False)
    training_pairs_node = OdometryTrainingPairsNode()
    rospy.on_shutdown(training_pairs_node.onShutdown)
    rospy.spin()

