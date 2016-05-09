#!/usr/bin/env python
import rospy
import rospkg, os
from duckietown_msgs.msg import Vsample, ThetaDotSample
import csv


class OdometryTrainingPairsNode(object):
    def __init__(self):
        self.node_name = rospy.get_name()

        #Init the files
        self.rpkg = rospkg.RosPack()
        self.v_sample_filename = self.setupParameter("~v_sample_filename", self.rpkg.get_path('visual_odometry') + '/tests/v_sample.csv')
        try:
            os.remove(self.v_sample_filename)
        except OSError:
            pass

        self.rpkg = rospkg.RosPack()
        self.theta_dot_sample_filename = self.setupParameter("~theta_dot_sample_filename", self.rpkg.get_path('visual_odometry') + '/tests/theta_dot_sample.csv')
        try:
            os.remove(self.theta_dot_sample_filename)
        except OSError:
            pass

        ## state vars

        ## publishers and subscribers
        self.sub_v_sample = rospy.Subscriber("odometry_training_pairs_node/v_sample", Vsample, self.vSampleCB)
        self.sub_theta_dot_sample = rospy.Subscriber("odometry_training_pairs_node/theta_dot_sample", ThetaDotSample, self.thetaDotSampleCB)
        rospy.loginfo('[%s] Initialized' % self.node_name)

    def setupParameter(self, param_name, default_value):
        value = rospy.get_param(param_name, default_value)
        rospy.set_param(param_name, value)  # Write to parameter server for transparancy
        rospy.loginfo("[%s] %s = %s " % (self.node_name, param_name, value))
        return value

    def vSampleCB(self, v_sample_msg):
        row = [v_sample_msg.d_L, \
               v_sample_msg.d_R, \
               v_sample_msg.dt, \
               v_sample_msg.theta_angle_pose_delta, \
               v_sample_msg.x_axis_pose_delta, \
               v_sample_msg.y_axis_pose_delta]

        with open(self.v_sample_filename, 'a+') as csvfile:
            writer = csv.writer(csvfile, delimiter=' ', quoting=csv.QUOTE_MINIMAL)
            writer.writerow(row)

    def thetaDotSampleCB(self, theta_dot_sample_msg):
        row = [theta_dot_sample_msg.d_L, \
               theta_dot_sample_msg.d_R, \
               theta_dot_sample_msg.dt, \
               theta_dot_sample_msg.theta_angle_pose_delta]

        with open(self.theta_dot_sample_filename, 'a+') as csvfile:
            writer = csv.writer(csvfile, delimiter=' ', quoting=csv.QUOTE_MINIMAL)
            print row
            writer.writerow(row)

    def onShutdown(self):
        rospy.loginfo("[%s] Shutdown."%self.node_name)

if __name__ == '__main__':
    rospy.init_node('odometry_training_pairs_2csv',anonymous=False)
    training_pairs_node_2csv = OdometryTrainingPairsNode()
    rospy.on_shutdown(training_pairs_node_2csv.onShutdown)
    rospy.spin()

