#!/usr/bin/env python
import rospy
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, Float32
from duckietown_msgs.msg import AprilTags, WheelsCmdStamped
from math import floor, atan2, pi, cos, sin
import time
import tf
import os

# Visual Odometry Line
# Author: Wyatt Ubellacker
# Inputs: SegmentList from line detector
# Outputs: Calculated Linear and Angular Velocity and uncertanity


class VisualOdometryAprilTagsNode(object):
    def __init__(self):
        self.node_name = "Visual Odometry April Tags"
        self.sub = rospy.Subscriber("~april_tags", AprilTags, self.processAprilTags)
        self.sub_wheels_cmd = rospy.Subscriber("~wheels_cmd", WheelsCmdStamped, self.processWheelsCmd)
        self.old_odometry_info = {}

        self.duty_L =0;
        self.duty_R =0;

    def setupParam(self,param_name,default_value):
        value = rospy.get_param(param_name,default_value)
        rospy.set_param(param_name,value) #Write to parameter server for transparancy
        rospy.loginfo("[%s] %s = %s " %(self.node_name,param_name,value))
        return value

    def processWheelsCmd(self,msg):
    	self.duty_L = msg.vel_left
    	self.duty_R = msg.vel_right

    def processAprilTags(self,msg):
        t_start = rospy.get_time()
        odometry_info = {}
        odometry_deltas = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        t = msg.header.stamp
        odometry_info['duty_L'] = self.duty_L
        odometry_info['duty_R'] = self.duty_R
        odometry_info['t'] = t.nsecs/1000000000.0; #nanoseconds to seconds
        count = 0
        for i in msg.detections:
        	tag_id = i.id
        	x = i.transform.translation.x
        	y = i.transform.translation.y
        	#convert quaternion to euler to get yaw
        	quaternion =(i.transform.rotation.x,
        	i.transform.rotation.y,
        	i.transform.rotation.z,
        	i.transform.rotation.w)
        	euler = tf.transformations.euler_from_quaternion(quaternion)
        	theta = euler[2] #yaw
        	transform = [-theta, -x, -y] #transform of robot is opposite of transform of tag
        	odometry_info[tag_id] = transform

        	#compare with odometry data from last step, take average if multiple tags

        	if tag_id in self.old_odometry_info:
        		count += 1.0
        		odometry_deltas[0] = self.old_odometry_info['duty_L'] #duty_L
        		odometry_deltas[1] = self.old_odometry_info['duty_R'] #duty_R
        		odometry_deltas[2] = odometry_info['t'] - self.old_odometry_info['t'] #dt
        		odometry_deltas[3] = odometry_deltas[3] + odometry_info[tag_id][0]- self.old_odometry_info[tag_id][0] #theta_angle_pose_delta
        		odometry_deltas[4] = odometry_deltas[4] + odometry_info[tag_id][1]- self.old_odometry_info[tag_id][1] #x_axis_pose_delta
        		odometry_deltas[5] = odometry_deltas[5] + odometry_info[tag_id][2]- self.old_odometry_info[tag_id][2] #y_axis_pose_delta
        		
    	self.old_odometry_info = odometry_info

    	#rospy.loginfo(odometry_info)

    	if count!=0:
    		odometry_deltas[3] = odometry_deltas[3]/float(count)
        	odometry_deltas[4] = odometry_deltas[4]/float(count)
        	odometry_deltas[5] = odometry_deltas[5]/float(count)
    		rospy.loginfo(odometry_deltas)
        	dir = os.path.dirname(os.path.realpath('__file__'))
        	filename = os.path.join(dir, '../duckietown/catkin_ws/src/visual_odometry_april_tags/training_data.txt')
        	f=open(filename,'a+')
        	np.savetxt(f,odometry_deltas, fmt='%-7.8f', newline=" ")
        	f.write('\n')
        	f.close()
    		#np.savetxt('/home/wyatt/duckietown/training_data.txt', odometry_deltas)
        #self.old_segment_list = segment_list_msg
        # print "time to process segments:"
        # print rospy.get_time() - t_start
    
    def onShutdown(self):
        rospy.loginfo("[VisualOdometryAprilTagsNode] Shutdown.")

if __name__ == '__main__': 
    rospy.init_node('visual_odometry_april_tags',anonymous=False)
    visual_odometry_april_tags_node = VisualOdometryAprilTagsNode()
    rospy.on_shutdown(visual_odometry_april_tags_node.onShutdown)
    rospy.spin()

