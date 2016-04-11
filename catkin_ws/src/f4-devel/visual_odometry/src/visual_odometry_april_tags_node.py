#!/usr/bin/env python
import rospy
import rospkg
import numpy as np
from duckietown_msgs.msg import AprilTags, WheelsCmdStamped
import tf
import tf.transformations as tr
import os


# Visual Odometry April Tags
# Author: Wyatt Ubellacker
# Inputs: april_tags, wheels_cmd
# Outputs: A text file training set


class VisualOdometryAprilTagsNode(object):
    def __init__(self):
        self.node_name = "visual_odometry_april_tags_node"
        self.sub = rospy.Subscriber("~april_tags", AprilTags, self.processAprilTags)
        self.sub_wheels_cmd = rospy.Subscriber("~wheels_cmd", WheelsCmdStamped, self.processWheelsCmd)

        #Init the file
        self.rpkg = rospkg.RosPack()
        self.filename = self.setupParameter("~filename", self.rpkg.get_path('kinematics') + '/include/kinematics/training_data.txt')
        try:
            os.remove(self.filename)
        except OSError:
            pass

        # To store the wheels_cmd
        self.wheels_cmd = WheelsCmdStamped()

        # Initialize prev values
        self.april_tags_prev = {}
        self.timestamp_prev = rospy.Time()
        self.wheels_cmd_prev = WheelsCmdStamped()

        rospy.loginfo("[%s] has started", self.node_name)

    def geometryTransformToMatrix(self, t):
        # Convert a geometry_msgs.Transform into a 4x4 homogeneous matrix
        tros = tf.TransformerROS()
        trans = (t.translation.x, t.translation.y, t.translation.z)
        rot = (t.rotation.x, t.rotation.y, t.rotation.z, t.rotation.w)
        return tros.fromTranslationRotation(trans, rot)

    def wheelsCmdAlmostEqual(self, x, y):
        # Return true if the two WheelsCmdStamped messages are the same within tolerance
        tol = 1e-6
        return abs(x.vel_left-y.vel_left) < tol and abs(x.vel_right-y.vel_right) < tol

    def setupParameter(self, param_name, default_value):
        value = rospy.get_param(param_name, default_value)
        rospy.set_param(param_name, value)  # Write to parameter server for transparancy
        rospy.loginfo("[%s] %s = %s " % (self.node_name, param_name, value))
        return value

    def processWheelsCmd(self, msg):
        self.wheels_cmd = msg

    def processAprilTags(self, msg):

        # Only process if the wheels_cmd hasn't changed since last time and the last time < cur time
        if self.wheelsCmdAlmostEqual(self.wheels_cmd, self.wheels_cmd_prev) and msg.header.stamp > self.timestamp_prev:
            deltas = []
            for tag in msg.detections:
                if tag.id in self.april_tags_prev:
                    Ta_r1 = self.geometryTransformToMatrix(self.april_tags_prev[tag.id].transform)
                    Ta_r2 = self.geometryTransformToMatrix(tag.transform)
                    Tr2_r1 = np.dot(Ta_r1, tr.inverse_matrix(Ta_r2))

                    # Get the angle, dx, and dy
                    theta = tr.euler_from_matrix(Tr2_r1)[2]
                    dx,dy = Tr2_r1[0:2,3]
                    deltas.append([theta, dx, dy])

            if deltas:
                # We found some matches, average them, and print
                deltas = np.mean(np.array(deltas),0).tolist()
                cmd = [self.wheels_cmd_prev.vel_left, self.wheels_cmd_prev.vel_right, (msg.header.stamp - self.timestamp_prev).to_sec()]
                sample = cmd+deltas
                rospy.loginfo(sample)

                f = open(self.filename, 'a+')
                np.savetxt(f, sample, fmt='%-7.8f', newline=" ")
                f.write('\n')
                f.close()
        else:
            err = "backwards time." if msg.header.stamp < self.timestamp_prev else "cmd changed"
            rospy.logwarn("Invalid interval %s", err)

        # Save the tags and wheels_cmd for next time
        for tag in msg.detections:
            self.april_tags_prev[tag.id] = tag
        self.timestamp_prev = msg.header.stamp
        self.wheels_cmd_prev = self.wheels_cmd

    def onShutdown(self):
        rospy.loginfo("[%s] Shutdown.", self.node_name)


if __name__ == '__main__':
    rospy.init_node('visual_odometry_april_tags', anonymous=False)
    visual_odometry_april_tags_node = VisualOdometryAprilTagsNode()
    rospy.on_shutdown(visual_odometry_april_tags_node.onShutdown)
    rospy.spin()
