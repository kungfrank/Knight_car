#!/usr/bin/env python
import rospy, rospkg
from duckietown_msgs.msg import Twist2DStamped, Pose2DStamped
from kinematics import Forward_kinematics
from numpy import *

# Position Filter Node
# Authors: Jason Pazis, Robert Katzschmann
# Inputs: velocity
# Outputs: pose

class PositionFilterNode(object):
    def __init__(self):
        self.node_name = 'position_filter_node'

        # Read parameters
        self.veh_name = self.setupParameter("~veh_name","megaman")
        
        fi_theta_dot_function = self.setupParameter('~fi_theta_dot_function_param', 'Duty_fi_theta_dot_naive')
        fi_v_function = self.setupParameter('~fi_v_function_param', 'Duty_fi_v_naive')
        theta_dot_weights = matrix(self.setupParameter('~theta_dot_weights_param', [-1.0]))
        v_weights = matrix(self.setupParameter('~v_weights_param', [1.0]))

        #Setup the forward kinematics model
        self.fk = Forward_kinematics.Forward_kinematics(fi_theta_dot_function, fi_v_function, matrix(theta_dot_weights), matrix(v_weights))

        #Setup the publisher and subscriber
        self.sub_velocity = rospy.Subscriber("~velocity", Twist2DStamped, self.velocityCallback)
        self.pub_pose = rospy.Publisher("~pose", Pose2DStamped, queue_size=1)

        #Keep track of the last known pose
        self.last_pose = Pose2DStamped()
        self.last_theta_dot = 0
        self.last_v = 0

        rospy.loginfo("[%s] has started", self.node_name)


    def velocityCallback(self, msg_velocity):
        if self.last_pose.header.stamp.to_sec() > 0:    # skip first frame
            delta_t = (msg_velocity.header.stamp - self.last_pose.header.stamp).to_sec()
            [theta_res, x_res, y_res] = self.fk.integrate_propagate(self.last_pose.theta, self.last_pose.x, self.last_pose.y,self.last_theta_dot, self.last_v, delta_t)
            self.last_pose.x = x_res
            self.last_pose.y = y_res
            self.last_pose.theta = theta_res

            # Stuff the new pose into a message and publish
            msg_pose = Pose2DStamped()
            msg_pose.header = msg_velocity.header
            msg_pose.header.frame_id = self.veh_name
            msg_pose.x = x_res
            msg_pose.y = y_res
            msg_pose.theta = theta_res
            self.pub_pose.publish(msg_pose)

        self.last_pose.header.stamp = msg_velocity.header.stamp
        self.last_theta_dot = msg_velocity.omega
        self.last_v = msg_velocity.v

    def setupParameter(self,param_name,default_value):
        value = rospy.get_param(param_name,default_value)
        rospy.set_param(param_name,value) #Write to parameter server for transparancy
        rospy.loginfo("[%s] %s = %s " %(self.node_name,param_name,value))
        return value


if __name__ == '__main__':
    rospy.init_node('position_filter_node', anonymous=False)
    position_filter_node = PositionFilterNode()
    rospy.spin()
