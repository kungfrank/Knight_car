#!/usr/bin/env python
import rospy
import numpy as np
import tf
from duckietown_msgs.msg import CarLanePose
from geometry_msgs.msg import PoseStamped

class vicon_for_car(object):
    def __init__(self):
        self.viconPose = PoseStamped()
        self.carPose = CarLanePose()
	self.pub_counter = 200
	self.sub_counter = 0

        # Setup Parameters
        self.pub_timestep = self.setupParam("~pub_timestep",0.02)  # 50 Hz

        # Publications
        self.pub_carPose = rospy.Publisher("~car_vicon", CarLanePose, queue_size=1)

        # Subscriptions
        self.sub_vicon_ = rospy.Subscriber("/duckiecar/pose", PoseStamped, self.cbPose)
        
        # timer
        self.pub_timer = rospy.Timer(rospy.Duration.from_sec(self.pub_timestep),self.pbCarPose)
        rospy.loginfo("[%s] Initialized " %(rospy.get_name()))

    def setupParam(self,para_name,default):
        if not rospy.has_param(para_name):
            rospy.set_param(para_name,default)
        return rospy.get_param(para_name)

    def cbPose(self,msg):
        self.viconPose = msg
	# debugging
	# self.sub_counter += 1
	# if self.sub_counter % 400 == 0:
	#	self.sub_counter = 1
	#	print "vicon_pose_msg", msg 


    def pbCarPose(self,event):
        self.carPose.cross_track_err = self.viconPose.pose.position.x
        euler_angles = tf.transformations.euler_from_quaternion( \
			[self.viconPose.pose.orientation.x, \
			 self.viconPose.pose.orientation.y, \
                         self.viconPose.pose.orientation.z, \
			 self.viconPose.pose.orientation.w] )
	self.carPose.heading_err = euler_angles[2]
        self.pub_carPose.publish(self.carPose)
	# debugging
	self.pub_counter += 1
	if self.pub_counter % 250 == 0:
		self.pub_counter = 1
		print "from vicon_for_car node, self.carPose"
		print  self.carPose


if __name__ == "__main__":
    rospy.init_node("vicon_for_car", anonymous=False)
    vicon_car_node = vicon_for_car()
    rospy.spin()
