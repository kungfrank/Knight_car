#!/usr/bin/env python
import rospy, rospkg
from duckietown_msgs.msg import WheelsCmdStamped, Twist2DStamped, Pose2DStamped
from kinematics import Forward_kinematics

# Forward Kinematics Node
# Author: Teddy Ort
# Inputs: 
# Outputs: 

class ForwardKinematicsNode(object):
    def __init__(self):
        self.node_name = 'forward_kinematics_node'
        #self.param_name = self.setupParam("~<param_name>", '<default value>')

        #Setup the forward kinematics model
        rp = rospkg.RosPack()
        path = rp.get_path('kinematics') + "/include/kinematics/"
        self.fk = Forward_kinematics.Forward_kinematics(path + "theta_dot_fi_function", path + "v_fi_function", path + "theta_dot_weights_file", path + "v_weights_file")

        #Setup the publisher and subscriber
        self.sub = rospy.Subscriber("~wheels_cmd", WheelsCmdStamped, self.wheelsCmdCallback)
        self.pub_velocity = rospy.Publisher("~velocity", Twist2DStamped, queue_size=10)
        self.pub_pose = rospy.Publisher("~pose", Pose2DStamped, queue_size=10)

        #Keep track of the last known pose
        self.last_pose = Pose2DStamped()
        rospy.loginfo("[%s] has started", self.node_name)

    def wheelsCmdCallback(self, msg_wheels_cmd):
        delta_t = (msg_wheels_cmd.header.stamp - self.last_pose.header.stamp).to_sec()
        [theta_dot, v, theta_delta, chord] = self.fk.evaluate_and_integrate(msg_wheels_cmd.vel_left, msg_wheels_cmd.vel_right, delta_t)
        [theta_res, x_res, y_res] = self.fk.propagate(self.last_pose.theta, self.last_pose.x, self.last_pose.y, theta_delta, chord)
        self.last_pose.x += x_res
        self.last_pose.y += y_res
        self.last_pose.theta += theta_res

        # Stuff the v and omega into a message and publish
        msg_velocity = Twist2DStamped()
        msg_velocity.header = msg_wheels_cmd.header
        msg_velocity.v = v
        msg_velocity.omega = theta_dot
        self.pub_velocity.publish(msg_velocity)

        # Stuff the new pose into a message and publish
        msg_pose = Pose2DStamped()
        msg_pose.header = msg_wheels_cmd.header
        msg_pose.x = x_res
        msg_pose.y = y_res
        msg_pose.theta = theta_res
        self.pub_pose.publish(msg_pose)

    def setupParam(self, param_name, default_value):
        value = rospy.get_param(param_name, default_value)
        rospy.set_param(param_name, value)  #Write to parameter server for transparancy
        rospy.loginfo("[%s] %s = %s " % (self.node_name, param_name, value))
        return value


if __name__ == '__main__':
    rospy.init_node('forward_kinematics_node', anonymous=False)
    forward_kinematics_node = ForwardKinematicsNode()
    rospy.spin()