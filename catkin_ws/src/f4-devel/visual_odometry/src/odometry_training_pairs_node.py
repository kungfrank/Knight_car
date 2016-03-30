#!/usr/bin/env python
import rospy
from duckietown_msgs.msg import StopLineReading, LanePose, WheelsCmdStamped, Vsample, ThetaDotSample
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
        self.old_lane_pose_msg = LanePose()

        ## publishers and subscribers
        self.sub_stop_lin_reading = rospy.Subscriber("~stop_line_reading", StopLineReading, self.stopLineCB)
        self.sub_lane_pose = rospy.Subscriber("~lane_pose", LanePose, self.lanePoseCB)
        self.sub_wheels_cmd_executed = rospy.Subscriber("~wheels_cmd_executed", WheelsCmdStamped, self.wheelsCmdCB)
        self.pub_v_sample = rospy.Publisher("~v_sample", Vsample, queue_size=1)
        self.pub_theta_dot_sample = rospy.Publisher("~theta_dot_sample", ThetaDotSample, queue_size=1)
        rospy.loginfo('[odometry_training_pairs_node] Initiated')

    def stopLineCB(self, stop_line_msg):
        if self.in_lane and self.current_wheels_cmd and stop_line_msg.header.stamp.secs - self.old_stop_line_msg.header.stamp.secs <= 1.0 and self.old_stop_line_msg:
            v_sample_msg = Vsample()
            v_sample_msg.d_L = self.current_wheels_cmd.vel_left
            v_sample_msg.d_R = self.current_wheels_cmd.vel_right
            #I think 
            v_sample_msg.dt = stop_line_msg.header.stamp.secs + stop_line_msg.header.stamp.nsecs/1e9 - self.old_stop_line_msg.header.stamp.secs - self.old_stop_line_msg.header.stamp.nsecs/1e9
            v_sample_msg.theta_angle_pose_delta = 0
            #x is the direction the robot is aiming, negative since it is measuring stop line x-coor old - new = -(new-old)
            v_sample_msg.x_axis_pose_delta = self.old_stop_line_msg.stop_line_point.x - stop_line_msg.stop_line_point.x
            #not sure what the y axis from the stop_line detection actually means
            v_sample_msg.y_axis_pose_delta = 0
            self.pub_v_sample.publish(v_sample_msg)
        #should reset the old msg regardless of whether the deltas were calculated or not
        self.old_stop_line_msg = stop_line_msg        

    def wheelsCmdCB(self, wheels_cmd_msg):
        self.current_wheels_cmd = wheels_cmd_msg

    def lanePoseCB(self, lane_pose_msg):
        self.lane_pose = lane_pose_msg
        self.in_lane = lane_pose_msg.in_lane
        if self.in_lane and self.current_wheels_cmd and lane_pose_msg.header.stamp.secs - self.old_lane_pose_msg.header.stamp.secs <= 1.0 and self.old_lane_pose_msg:
            theta_dot_sample_msg = ThetaDotSample()
            theta_dot_sample_msg.d_L = self.current_wheels_cmd.vel_left
            theta_dot_sample_msg.d_R = self.current_wheels_cmd.vel_right
            theta_dot_sample_msg.dt = lane_pose_msg.header.stamp.secs + lane_pose_msg.header.stamp.nsecs/1e9 - self.old_lane_pose_msg.header.stamp.secs - self.old_lane_pose_msg.header.stamp.nsecs/1e9
            #new - old since measure robot angle
            theta_dot_sample_msg.theta_angle_pose_delta = lane_pose_msg.phi - self.old_lane_pose_msg.phi
            #is actually (delta d)*cos(phi), but is probably pretty noisy since we get error from both d and phi
            #theta_dot_sample_msg.x_axis_pose_delta = (lane_pose_msg.d - self.old_lane_pose_msg.d)*cos(lane_pose_msg.phi)
            #theta_dot_sample_msg.y_axis_pose_delta = 0
            self.pub_theta_dot_sample.publish(theta_dot_sample_msg)
        self.old_lane_pose_msg = lane_pose_msg

    def onShutdown(self):
        rospy.loginfo("[odometry_training_pairs_node] Shutdown.")

if __name__ == '__main__':
    rospy.init_node('odometry_training_pairs',anonymous=False)
    training_pairs_node = OdometryTrainingPairsNode()
    rospy.on_shutdown(training_pairs_node.onShutdown)
    rospy.spin()

