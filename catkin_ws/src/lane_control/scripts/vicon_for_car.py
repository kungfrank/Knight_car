#!/usr/bin/env python
import rospy
import numpy as np
import tf
from duckietown_msgs.msg import LaneReading
from geometry_msgs.msg import PoseStamped

class vicon_for_car(object):
    def __init__(self):
        self.node_name = rospy.get_name()
        self.veh_name = self.setupParameter("~veh_name","duckiecar")
        self.vicon_pose = None
        self.lane_reading = LaneReading()

        # self.pub_counter = 200
        # self.sub_counter = 0

        # Setup Parameters
        self.pub_timestep = self.setupParameter("~pub_timestep",0.02)  # 50 Hz

        # Publications
        self.pub_lane_reading = rospy.Publisher("~lane_reading", LaneReading, queue_size=1)

        # Subscriptions
        self.sub_vicon = rospy.Subscriber(self.veh_name+"/pose", PoseStamped, self.cbPose)
        
        # timer
        self.pub_timer = rospy.Timer(rospy.Duration.from_sec(self.pub_timestep),self.cbPubPose)
        rospy.loginfo("[%s] Initialized " %(rospy.get_name()))

    def setupParameter(self,param_name,default_value):
        value = rospy.get_param(param_name,default_value)
        rospy.set_param(param_name,value) #Write to parameter server for transparancy
        rospy.loginfo("[%s] %s = %s " %(self.node_name,param_name,value))
        return value

    def cbPose(self,msg):
        self.vicon_pose = msg
        # debugging
        # self.sub_counter += 1
        # if self.sub_counter % 400 == 0:
        #   self.sub_counter = 1
        #   print "vicon_pose_msg", msg 

    def cbPubPose(self,event):
        if self.vicon_pose is None:
            rospy.loginfo("[%s] No vicon readings yet." %(self.node_name))
            return

        quat = self.vicon_pose.pose.orientation
        euler_angles = tf.transformations.euler_from_quaternion([quat.x, quat.y,quat.z,quat.w])

        self.lane_reading.y = self.vicon_pose.pose.position.x
        self.lane_reading.phi = euler_angles[2]
        self.lane_reading.header.stamp = self.vicon_pose.header.stamp
        self.pub_lane_reading.publish(self.lane_reading)
    
        # # debugging
        # self.pub_counter += 1
        # if self.pub_counter % 250 == 0:
        #     self.pub_counter = 1
        #     print "from vicon_for_car node, self.lane_reading"
        #     print  self.lane_reading

if __name__ == "__main__":
    rospy.init_node("vicon_for_car", anonymous=False)
    vicon_car_node = vicon_for_car()
    rospy.spin()
