#!/usr/bin/env python
import rospy
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from duckietown_msgs.msg import SegmentList, Segment, Pixel, LanePose
from scipy.stats import multivariate_normal
from math import floor, atan2, pi, cos, sin
import time

# Lane Filter Node
# Author: Liam Paull
# Inputs: SegmentList from line detector
# Outputs: LanePose - the d (lateral displacement) and phi (relative angle) of the car in the lane
# For more info on algorithm and parameters please refer to the google doc: https://drive.google.com/open?id=0B49dGT7ubfmSX1k5ZVN1dEU4M2M


class LaneFilterNode(object):
    def __init__(self):
        self.node_name = "Lane Filter"
        self.mean_0 = [self.setupParam("~mean_d_0",0) , self.setupParam("~mean_phi_0",0)]
        self.cov_0  = [ [self.setupParam("~sigma_d_0",0.1) , 0] , [0, self.setupParam("~sigma_phi_0",0.01)] ] 
        self.delta_d     = self.setupParam("~delta_d",0.02) # in meters
        self.delta_phi   = self.setupParam("~delta_phi",0.02) # in radians
        self.d_max       = self.setupParam("~d_max",0.5)
        self.d_min       = self.setupParam("~d_min",-0.7)
        self.phi_min     = self.setupParam("~phi_min",-pi/2)
        self.phi_max     = self.setupParam("~phi_max",pi/2)
        self.cov_v       = self.setupParam("~cov_v",0.5) # linear velocity "input"
        self.cov_omega   = self.setupParam("~cov_omega",0.01) # angular velocity "input"
        self.linewidth_white = self.setupParam("~linewidth_white",0.04)
        self.linewidth_yellow = self.setupParam("~linewidth_yellow",0.02)
        self.lanewidth        = self.setupParam("~lanewidth",0.4)
        self.max_entropy = self.setupParam("~max_entropy", 0.04) # nats

        self.d,self.phi = np.mgrid[self.d_min:self.d_max:self.delta_d,self.phi_min:self.phi_max:self.delta_phi]
        self.beliefRV=np.empty(self.d.shape)
        self.initializeBelief()
        self.lanePose = LanePose()
        self.lanePose.d=self.mean_0[0]
        self.lanePose.phi=self.mean_0[1]
        self.sub = rospy.Subscriber("~segment_list", SegmentList, self.processSegments)
        # self.sub = rospy.Subscriber("~velocity",
        self.pub_lane_pose  = rospy.Publisher("~lane_pose", LanePose, queue_size=1)
        self.pub_belief_img = rospy.Publisher("~belief_img", Image, queue_size=1)

    def setupParam(self,param_name,default_value):
        value = rospy.get_param(param_name,default_value)
        rospy.set_param(param_name,value) #Write to parameter server for transparancy
        rospy.loginfo("[%s] %s = %s " %(self.node_name,param_name,value))
        return value

    def processSegments(self,segment_list_msg):
        t_start = rospy.get_time()
        self.propagateBelief()
        # initialize measurement likelihood
        measurement_likelihood = np.zeros(self.d.shape)
        for segment in segment_list_msg.segments:
            if segment.color != segment.WHITE and segment.color != segment.YELLOW:
                continue
            if segment.points[0].x < 0 or segment.points[1].x < 0:
                continue
            # todo eliminate the white segments that are on the other side of the road
            d_i,phi_i,l_i = self.generateVote(segment)
            if d_i > self.d_max or d_i < self.d_min or phi_i < self.phi_min or phi_i>self.phi_max:
                continue
            i = floor((d_i - self.d_min)/self.delta_d)
            j = floor((phi_i - self.phi_min)/self.delta_phi)
            measurement_likelihood[i,j] = measurement_likelihood[i,j] +  1/l_i
        if np.linalg.norm(measurement_likelihood) == 0:
            return
        measurement_likelihood = measurement_likelihood/np.linalg.norm(measurement_likelihood)
        #self.updateBelief(measurement_likelihood)
        self.beliefRV = measurement_likelihood
        # TODO entropy test:
        #print self.beliefRV.argmax()
        
        maxids = np.unravel_index(self.beliefRV.argmax(),self.beliefRV.shape)
        self.lanePose.header.stamp = segment_list_msg.header.stamp
        self.lanePose.d = self.d_min + maxids[0]*self.delta_d
        self.lanePose.phi = self.phi_min + maxids[1]*self.delta_phi
        self.lanePose.status = self.lanePose.NORMAL

        # publish the belief image
        bridge = CvBridge()
        belief_img = bridge.cv2_to_imgmsg((255*self.beliefRV).astype('uint8'), "mono8")
        belief_img.header.stamp = segment_list_msg.header.stamp
        self.pub_lane_pose.publish(self.lanePose)
        self.pub_belief_img.publish(belief_img)
        print "time to process segments:"
        print rospy.get_time() - t_start


    def initializeBelief(self):
        pos = np.empty(self.d.shape + (2,))
        pos[:,:,0]=self.d
        pos[:,:,1]=self.phi
        self.cov_0
        RV = multivariate_normal(self.mean_0,self.cov_0)
        self.beliefRV=RV.pdf(pos)

    def propagateBelief(self):
        # starting with option 1 (don't read linear and angular velocity)
        # even simpler.. do nothing
        return

    def updateBelief(self,measurement_likelihood):
        self.beliefRV=np.multiply(self.beliefRV,measurement_likelihood)
        self.beliefRV=self.beliefRV/np.linalg.norm(self.beliefRV)

    def generateVote(self,segment):
        p1 = np.array([segment.points[0].x, segment.points[0].y]) 
        p2 = np.array([segment.points[1].x, segment.points[1].y])
        t_hat = (p2-p1)/np.linalg.norm(p2-p1)
        n_hat = np.array([-t_hat[1],t_hat[0]])
        d1 = np.inner(n_hat,p1)
        d2 = np.inner(n_hat,p2)
        l1 = np.inner(t_hat,p1)
        l2 = np.inner(t_hat,p2)
        if (l1 < 0):
            l1 = -l1;
        if (l2 < 0):
            l2 = -l2;
        l_i = (l1+l2)/2
        d_i = (d1+d2)/2
        phi_i = np.arcsin(t_hat[1])
        if segment.color == segment.WHITE: # right lane is white
            if(p1[0] > p2[0]): # right edge of white lane
                d_i = d_i - self.linewidth_white
            else: # left edge of white lane
                d_i = - d_i
                phi_i = -phi_i
            d_i = d_i - self.lanewidth/2
        elif segment.color == segment.YELLOW: # left lane is yellow
            if (p2[0] > p1[0]): # left edge of yellow lane
                d_i = d_i - self.linewidth_yellow
                phi_i = -phi_i
            else: # right edge of white lane
                d_i = -d_i
            d_i =  self.lanewidth/2 - d_i
        return d_i, phi_i, l_i
    
    def onShutdown(self):
        rospy.loginfo("[LaneFilterNode] Shutdown.")

if __name__ == '__main__': 
    rospy.init_node('lane_filter',anonymous=False)
    lane_filter_node = LaneFilterNode()
    rospy.on_shutdown(lane_filter_node.onShutdown)
    rospy.spin()

