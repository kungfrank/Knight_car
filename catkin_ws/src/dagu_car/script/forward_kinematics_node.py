#!/usr/bin/env python
import rospy
from duckietown_msgs.msg import WheelsCmdStamped, Twist2DStamped
from duckietown_msgs.srv import SetValueRequest, SetValueResponse, SetValue
from std_srvs.srv import EmptyRequest, EmptyResponse, Empty
from numpy import *
import rospkg
import yaml
import time
import os.path


# Forward Kinematics Node
# Authors: Robert Katzschmann
# Inputs: wheels cmd
# Outputs: velocity

class ForwardKinematicsNode(object):
    def __init__(self):
        # Get node name and vehicle name
        self.node_name = rospy.get_name()
        self.veh_name = self.node_name.split("/")[1]

        # Set parameters using yaml file
        self.readParamFromFile()

        # Set local variable by reading parameters
        self.gain = self.setup_parameter("~gain", 1.0)
        self.trim = self.setup_parameter("~trim", 0.0)
        self.baseline = self.setup_parameter("~baseline", 0.1)
        self.radius = self.setup_parameter("~wheel_radius", 0.0318)

        # Prepare services
        self.srv_set_gain = rospy.Service("~set_gain", SetValue, self.cbSrvSetGain)
        self.srv_set_trim = rospy.Service("~set_trim", SetValue, self.cbSrvSetTrim)
        self.srv_set_baseline = rospy.Service("~set_baseline", SetValue, self.cbSrvSetBaseline)
        self.srv_save = rospy.Service("~save_calibration", Empty, self.cbSrvSaveCalibration)

        # Setup the publisher and subscribers
        self.pub_velocity = rospy.Publisher("~velocity", Twist2DStamped, queue_size=1)
        self.sub_wheels_cmd = rospy.Subscriber("~wheels_cmd", WheelsCmdStamped, self.wheels_cmd_callback)
        rospy.loginfo("[%s] Initialized.", self.node_name)
        self.printValues()

    def readParamFromFile(self):
        # Check file existence
        fname = self.getFilePath(self.veh_name)
        # Use default.yaml if file doesn't exsit
        if not os.path.isfile(fname):
            rospy.logwarn("[%s] %s does not exist. Using default.yaml." % (self.node_name, fname))
            fname = self.getFilePath("default")

        with open(fname, 'r') as in_file:
            try:
                yaml_dict = yaml.load(in_file)
            except yaml.YAMLError as exc:
                rospy.logfatal("[%s] YAML syntax error. File: %s fname. Exc: %s" % (self.node_name, fname, exc))
                rospy.signal_shutdown()
                return

        # Set parameters using value in yaml file
        if yaml_dict is None:
            # Empty yaml file
            return
        for param_name in ["gain", "trim", "baseline", "radius"]:
            param_value = yaml_dict.get(param_name)
            if param_name is not None:
                rospy.set_param("~" + param_name, param_value)
            else:
                # Skip if not defined, use default value instead.
                pass

    def getFilePath(self, name):
        rospack = rospkg.RosPack()
        return rospack.get_path('duckietown') + '/config/baseline/calibration/kinematics/' + name + ".yaml"

    def saveCalibration(self):
        # Write to yaml
        data = {
            "calibration_time": time.strftime("%Y-%m-%d-%H-%M-%S"),
            "gain": self.gain,
            "trim": self.trim,
            "baseline": self.baseline,
            "radius": self.radius,
        }

        # Write to file
        file_name = self.getFilePath(self.veh_name)
        with open(file_name, 'w') as outfile:
            outfile.write(yaml.dump(data, default_flow_style=False))
        # Printout
        self.printValues()
        rospy.loginfo("[%s] Saved to %s" % (self.node_name, file_name))

    def cbSrvSaveCalibration(self, req):
        self.saveCalibration()
        return EmptyResponse()

    def cbSrvSetGain(self, req):
        self.gain = req.value
        self.printValues()
        return SetValueResponse()

    def cbSrvSetTrim(self, req):
        self.trim = req.value
        self.printValues()
        return SetValueResponse()

    def cbSrvSetBaseline(self, req):
        self.baseline = req.value
        self.printValues()
        return SetValueResponse()

    def printValues(self):
        rospy.loginfo("[%s] gain: %s trim: %s baseline: %s" % (self.node_name, self.gain, self.trim, self.baseline))

    def wheels_cmd_callback(self, msg_wheels_cmd):
        # compute duty cycle gain
        k_r = 1 / self.radius
        k_l = 1 / self.radius

        k_r_inv = (self.gain + self.trim) / k_r
        k_l_inv = (self.gain - self.trim) / k_l

        # Conversion from motor duty to motor rotation rate
        omega_r = msg_wheels_cmd.vel_right / k_r_inv
        omega_l = msg_wheels_cmd.vel_left / k_l_inv

        # Compute linear and angular velocity of the platform
        v = (self.radius * omega_r + self.radius * omega_l) / 2.0
        omega = (self.radius * omega_r - self.radius * omega_l) / self.baseline

        # Stuff the v and omega into a message and publish
        msg_velocity = Twist2DStamped()
        msg_velocity.header = msg_wheels_cmd.header
        msg_velocity.v = v
        msg_velocity.omega = omega
        self.pub_velocity.publish(msg_velocity)

    def setup_parameter(self, param_name, default_value):
        value = rospy.get_param(param_name, default_value)
        # Write to parameter server for transparency
        rospy.set_param(param_name, value)
        rospy.loginfo("[%s] %s = %s " %(self.node_name, param_name, value))
        return value

if __name__ == '__main__':
    rospy.init_node('forward_kinematics_node', anonymous=False)
    forward_kinematics_node = ForwardKinematicsNode()
    rospy.spin()
