#!/usr/bin/env python
import rospy
import rospkg, os
import csv
from duckietown_msgs.msg import LanePose, StopLineReading, WheelsCmdStamped


class LaneFilter2csv(object):
    def __init__(self):
        self.node_name = rospy.get_name()

        ## init files
        self.rpkg = rospkg.RosPack()
        self.wheels_cmd_file = self.setupParameter("~wheels_cmd_file", self.rpkg.get_path('visual_odometry') +
                                                   '/tests/wheels_cmd.csv')
        try:
            os.remove(self.wheels_cmd_file)
        except OSError:
            pass

        self.rpkg = rospkg.RosPack()
        self.stop_line_file = self.setupParameter("~stop_line_file", self.rpkg.get_path('visual_odometry') +
                                                   '/tests/stop_line.csv')
        try:
            os.remove(self.stop_line_file)
        except OSError:
            pass

        self.rpkg = rospkg.RosPack()
        self.lane_pose_file = self.setupParameter("~lane_pose_file", self.rpkg.get_path('visual_odometry') +
                                                   '/tests/lane_pose.csv')
        try:
            os.remove(self.lane_pose_file)
        except OSError:
            pass

        ## subscribers
        self.sub_wheels_cmd = rospy.Subscriber("~wheels_cmd", WheelsCmdStamped, self.wheelsCmdCB)
        self.sub_stop_line = rospy.Subscriber("~stop_line_reading", StopLineReading, self.stopLineCB)
        self.sub_lane_pose = rospy.Subscriber("~lane_pose", LanePose, self.lanePoseCB)
        rospy.loginfo('[%s] Initialized' % self.node_name)

    def setupParameter(self, param_name, default_value):
        value = rospy.get_param(param_name, default_value)
        rospy.set_param(param_name, value)  # Write to parameter server for transparancy
        rospy.loginfo("[%s] %s = %s " % (self.node_name, param_name, value))
        return value

    def wheelsCmdCB(self, wheels_cmd_msg):
        time = wheels_cmd_msg.header.stamp.to_sec()
        vel_left = wheels_cmd_msg.vel_left
        vel_right = wheels_cmd_msg.vel_right
        row = [time, vel_left, vel_right]
        print row
        with open(self.wheels_cmd_file, 'a+') as csvfile:
            writer = csv.writer(csvfile, delimiter=' ', quoting=csv.QUOTE_MINIMAL)
            writer.writerow(row)

    def stopLineCB(self, stop_line_msg):
        time = stop_line_msg.header.stamp.to_sec()
        x = stop_line_msg.stop_line_point.x
        row = [time, x]
        with open(self.stop_line_file, 'a+') as csvfile:
            writer = csv.writer(csvfile, delimiter=' ', quoting=csv.QUOTE_MINIMAL)
            writer.writerow(row)

    def lanePoseCB(self, lane_pose_msg):
        time = lane_pose_msg.header.stamp.to_sec()
        phi = lane_pose_msg.phi
        d = lane_pose_msg.d
        sigma_phi = lane_pose_msg.sigma_phi
        sigma_d = lane_pose_msg.sigma_d
        row = [time, phi, d, sigma_phi, sigma_d]
        with open(self.lane_pose_file, 'a+') as csvfile:
            writer = csv.writer(csvfile, delimiter=' ', quoting=csv.QUOTE_MINIMAL)
            writer.writerow(row)

    def onShutdown(self):
        rospy.loginfo("[%s] Shutdown."%self.node_name)

if __name__ == '__main__':
    rospy.init_node('lane_filter_2csv',anonymous=False)
    lane_filter_2csv = LaneFilter2csv()
    rospy.on_shutdown(lane_filter_2csv.onShutdown)
    rospy.spin()

