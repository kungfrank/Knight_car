#!/usr/bin/env python
import rospy, rospkg
from duckietown_msgs.msg import WheelsCmdStamped, Twist2DStamped, Pose2DStamped, KinematicsWeights
from kinematics import Forward_kinematics
from numpy import *

# Forward Kinematics Node
# Authors: Teddy Ort, Jason Pazis
# Inputs: 
# Outputs: 

class ForwardKinematicsNode(object):
    def __init__(self):
        self.node_name = 'forward_kinematics_node'

        # Read parameters
        # self.veh_name = self.setupParameter("~veh_name","megaman")
        fi_theta_dot_function = self.setupParameter('~fi_theta_dot_function_param', 'Duty_fi_theta_dot_naive')
        fi_v_function = self.setupParameter('~fi_v_function_param', 'Duty_fi_v_naive')
        theta_dot_weights = matrix(self.setupParameter('~theta_dot_weights_param', [-1.0]))
        v_weights = matrix(self.setupParameter('~v_weights_param', [1.0]))
        self.filename = self.setupParameter("~FIfile","FIfile")

        #Setup the forward kinematics model
        self.fk = Forward_kinematics.Forward_kinematics(fi_theta_dot_function, fi_v_function, theta_dot_weights, v_weights)

        #Setup the publisher and subscribers
        self.pub_velocity = rospy.Publisher("~velocity", Twist2DStamped, queue_size=1)
        self.sub = rospy.Subscriber("~wheels_cmd", WheelsCmdStamped, self.wheelsCmdCallback)
        self.sub_theta_dot_weights = rospy.Subscriber("~theta_dot_weights", KinematicsWeights, self.thedaDotWeightsCallback)
        self.sub_v_weights = rospy.Subscriber("~v_weights", KinematicsWeights, self.vWeightsCallback)

        rospy.loginfo("[%s] has started", self.node_name)

    def wheelsCmdCallback(self, msg_wheels_cmd):
        [theta_dot, v] = self.fk.evaluate(matrix(msg_wheels_cmd.vel_left), matrix(msg_wheels_cmd.vel_right))

        # Stuff the v and omega into a message and publish
        msg_velocity = Twist2DStamped()
        msg_velocity.header = msg_wheels_cmd.header
        #msg_velocity.header.frame_id = self.veh_name

        msg_velocity.v = v
        msg_velocity.omega = theta_dot
        self.pub_velocity.publish(msg_velocity)


    def thedaDotWeightsCallback(self, msg):
        # update theta_dot_weights for the forward kinematics model
        self.fk.theta_dot_weights = matrix(msg.weights)

    def vWeightsCallback(self, msg):
        # update v_weights for the forward kinematics model
        self.fk.v_weights = matrix(msg.weights)

    def setupParameter(self,param_name,default_value):
        value = rospy.get_param(param_name,default_value)
        rospy.set_param(param_name,value) #Write to parameter server for transparancy
        rospy.loginfo("[%s] %s = %s " %(self.node_name,param_name,value))
        return value


if __name__ == '__main__':
    rospy.init_node('forward_kinematics_node', anonymous=False)
    forward_kinematics_node = ForwardKinematicsNode()
    rospy.spin()
