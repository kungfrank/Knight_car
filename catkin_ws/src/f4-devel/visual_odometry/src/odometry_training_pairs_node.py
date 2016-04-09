#!/usr/bin/env python
import rospy
from duckietown_msgs.msg import StopLineReading, LanePose, WheelsCmdStamped, Vsample, ThetaDotSample
from collections import deque
import math


class OdometryTrainingPairsNode(object):
    def __init__(self):
        self.node_name = rospy.get_name()

        ## Parameters
        self.stop_line_max_age = self.setupParameter("~stop_line_max_age",1.0) # [sec] don't use stop_line_reading msg pairs more than this far apart
        self.lane_pose_max_age = self.setupParameter("~lane_pose_max_age", 1.0) # [sec] don't use lane_pose msg pairs more than this far apart
        self.wheels_cmd_max_age = self.setupParameter("~wheels_cmd_max_age", 2.0) # [sec] throw away wheels_cmd's older than this

        ## state vars
        self.in_lane = False
        self.current_wheels_cmd = WheelsCmdStamped()
        self.old_stop_line_msg = StopLineReading()
        self.old_lane_pose_msg = LanePose()
        self.cmd_buffer = deque() # buffer of wheel commands

        self.v_sample_msg = Vsample() # global variable because different parts of this are set in different callbacks

        ## publishers and subscribers
        self.sub_stop_line_reading = rospy.Subscriber("~stop_line_reading", StopLineReading, self.stopLineCB)
        self.sub_lane_pose = rospy.Subscriber("~lane_pose", LanePose, self.lanePoseCB)
        self.sub_wheels_cmd_executed = rospy.Subscriber("~wheels_cmd_executed", WheelsCmdStamped, self.wheelsCmdCB)
        self.pub_v_sample = rospy.Publisher("~v_sample", Vsample, queue_size=1)
        self.pub_theta_dot_sample = rospy.Publisher("~theta_dot_sample", ThetaDotSample, queue_size=1)
        rospy.loginfo('[%s] Initialized' % self.node_name)

    def setupParameter(self,param_name,default_value):
        value = rospy.get_param(param_name,default_value)
        rospy.set_param(param_name,value) #Write to parameter server for transparancy
        rospy.loginfo("[%s] %s = %s " %(self.node_name,param_name,value))
        return value

    def stopLineCB(self, stop_line_msg):
        # only execute if in lane and if the old message exists and is not too old
        if self.in_lane and hasattr(self.old_stop_line_msg, 'header') and (stop_line_msg.header.stamp - self.old_stop_line_msg.header.stamp).to_sec() <= self.stop_line_max_age:
            (d_L, d_R) = self.findMatchingDuties(stop_line_msg.header.stamp)
            self.v_sample_msg.d_L = d_L
            self.v_sample_msg.d_R = d_R
            self.v_sample_msg.dt = (stop_line_msg.header.stamp - self.old_stop_line_msg.header.stamp).to_sec()
            # x is the direction the robot is aiming, negative since it is measuring stop line x-coor old - new = -(new-old)
            self.v_sample_msg.x_axis_pose_delta = (self.old_stop_line_msg.stop_line_point.x - stop_line_msg.stop_line_point.x)/math.cos(self.lane_pose.phi)
            self.pub_v_sample.publish(self.v_sample_msg)
        # reset the old msg regardless of whether the deltas were calculated or not
        self.old_stop_line_msg = stop_line_msg

    def wheelsCmdCB(self, wheels_cmd_msg):
        # append new cmd to the right of the deque
        self.cmd_buffer.append(wheels_cmd_msg)
        # throw away cmd_msgs older than max_age seconds to keep the buffer small
        while (wheels_cmd_msg.header.stamp - self.cmd_buffer[0].header.stamp).to_sec() > self.wheels_cmd_max_age:
            self.cmd_buffer.popleft()

    def findMatchingDuties(self, new_stamp):
        # rospy.loginfo("[%s]cmd_buffer length: %s"%(self.node_name, (len(self.cmd_buffer))))
        # if no cmds have been received, return (0, 0). This shouldn't happen unless there's no wheels_cmd publiser
        if not self.cmd_buffer:
            return (0, 0)
        cmd = None
        # delete cmds in the buffer older than new_stamp
        while len(self.cmd_buffer) and (new_stamp - self.cmd_buffer[0].header.stamp).to_sec() > 0:
            cmd = self.cmd_buffer.popleft()
        if cmd:
            # append last popped cmd back into buffer on the left to make sure we always have at least one cmd
            self.cmd_buffer.appendleft(cmd)
        else:
            # if the oldest cmd in the buffer is not older than the new stamp, take the oldest
            cmd = self.cmd_buffer[0]
        return (cmd.vel_left, cmd.vel_right)

    def lanePoseCB(self, lane_pose_msg):
        self.lane_pose = lane_pose_msg
        self.in_lane = lane_pose_msg.in_lane
        # only execute if in_lane and if the old message exists and is not too old
        if self.in_lane and hasattr(self.old_lane_pose_msg, 'header') and (lane_pose_msg.header.stamp - self.old_lane_pose_msg.header.stamp).to_sec() <= self.lane_pose_max_age:
            theta_dot_sample_msg = ThetaDotSample()
            (d_L, d_R) = self.findMatchingDuties(lane_pose_msg.header.stamp)
            theta_dot_sample_msg.d_L = d_L
            theta_dot_sample_msg.d_R = d_R
            # theta_dot_sample_msg.dt = (lane_pose_msg.header.stamp.secs + lane_pose_msg.header.stamp.nsecs/1e9) - (self.old_lane_pose_msg.header.stamp.secs + self.old_lane_pose_msg.header.stamp.nsecs/1e9)
            theta_dot_sample_msg.dt = (lane_pose_msg.header.stamp - self.old_lane_pose_msg.header.stamp).to_sec()
            # rospy.loginfo("theta_dot_sample_msg.dt = %s"%theta_dot_sample_msg.dt)
            #new - old since measure robot angle
            theta_dot_sample_msg.theta_angle_pose_delta = lane_pose_msg.phi - self.old_lane_pose_msg.phi
            #is actually (delta d)*cos(phi), but is probably pretty noisy since we get error from both d and phi
            #theta_dot_sample_msg.x_axis_pose_delta = (lane_pose_msg.d - self.old_lane_pose_msg.d)*cos(lane_pose_msg.phi)
            #theta_dot_sample_msg.y_axis_pose_delta = 0
            self.pub_theta_dot_sample.publish(theta_dot_sample_msg)

            self.v_sample_msg.theta_angle_pose_delta = theta_dot_sample_msg.theta_angle_pose_delta
            #may need to swap signs on this one
            self.v_sample_msg.y_axis_pose_delta = lane_pose_msg.d - self.old_lane_pose_msg.d

        self.old_lane_pose_msg = lane_pose_msg

    def onShutdown(self):
        rospy.loginfo("[%s] Shutdown."%self.node_name)

if __name__ == '__main__':
    rospy.init_node('odometry_training_pairs',anonymous=False)
    training_pairs_node = OdometryTrainingPairsNode()
    rospy.on_shutdown(training_pairs_node.onShutdown)
    rospy.spin()

