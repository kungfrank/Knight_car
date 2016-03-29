#!/usr/bin/env python
import rospy
from duckietown_msgs.msg import WheelsCmdStamped
from dagu_car.dagu_wheels_driver import DaguWheelsDriver
from sensor_msgs.msg import CameraInfo

class WheelsDriverAdvancedNode(object):
    def __init__(self):
        self.node_name = rospy.get_name()
        rospy.loginfo("[%s] Initializing " %(self.node_name))

        #internal state of left and right velocity
        self.vel_left = 0.
        self.vel_right = 0.

        # Setup publishers
        self.driver = DaguWheelsDriver()
        #add publisher for wheels command wih execution time
        self.pub_wheels_cmd = rospy.Publisher("~wheels_cmd_executed",WheelsCmdStamped, self.cbWheelsCmd, queue_size=1)

        # Setup subscribers
        self.sub_topic = rospy.Subscriber("~wheels_cmd", WheelsCmdStamped, self.cbWheelsCmd, queue_size=1)
        # subscribe to camera info
        self.sub_camerainfo =  rospy.Subscriber("~camera_info", CameraInfo, self.cbCamInfo, queue_size=1)


    def setupParam(self,param_name,default_value):
        value = rospy.get_param(param_name,default_value)
        rospy.set_param(param_name,value) #Write to parameter server for transparancy
        rospy.loginfo("[%s] %s = %s " %(self.node_name,param_name,value))
        return value
    
    def cbCamInfo(self,msg):
        #synchronization between video frames and set wheelspeed
        self.driver.setWheelsSpeed(left=self.vel_left,right=self.vel_right)

        # Put the wheel commands in a message and publish
        msg_wheels_cmd = WheelsCmdStamped()
        msg_wheels_cmd.header = msg.header

        msg_wheels_cmd.header.stamp = rospy.get_rostime()  # Record the time the command was given to the wheels_driver
        msg_wheels_cmd.vel_left = self.vel_left
        msg_wheels_cmd.vel_right = self.vel_right
        self.pub_wheels_cmd.publish(msg_wheels_cmd)

    def cbWheelsCmd(self,msg):
        #to allow synchronization just store commanded values for now     
        self.vel_left = msg.vel_left
        self.vel_right = msg.vel_right
    
    def on_shutdown(self):
        self.driver.setWheelsSpeed(left=0.0,right=0.0)
        rospy.loginfo("[%s] Shutting down."%(rospy.get_name()))

if __name__ == '__main__':
    # Initialize the node with rospy
    rospy.init_node('wheels_driver_sync_node', anonymous=False)
    # Create the DaguCar object
    node = WheelsDriverAdvancedNode()
    # Setup proper shutdown behavior 
    rospy.on_shutdown(node.on_shutdown)
    # Keep it spinning to keep the node alive
    rospy.spin()
