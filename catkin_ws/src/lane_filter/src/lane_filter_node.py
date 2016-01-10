#!/usr/bin/env python
import rospy
import numpy as np
from duckietown_msgs.msg import SegmentList, Segment, Pixel, LanePose
from scipy.stats import multivariate_normal
from math import floor, atan2, pi

# Lane Filter Node
# Author: Liam Paull
# Inputs: SegmentList from line detector
# Outputs: LanePose - the d (lateral displacement) and phi (relative angle) of the car in the lane
# For more info on algorithm and parameters please refer to the google doc: https://drive.google.com/open?id=0B49dGT7ubfmSX1k5ZVN1dEU4M2M


class LaneFilterNode(object):
    def __init__(self):
        self.node_name = "Lane Filter"
        self.sub = rospy.Subscriber("~segment_list", SegmentList, self.processSegments)
        # self.sub = rospy.Subscriber("~velocity",
        self.pub_lane_pose = rospy.Publisher("~lane_pose", LanePose)
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
        


    def setupParam(self,param_name,default_value):
        value = rospy.get_param(param_name,default_value)
        rospy.set_param(param_name,value) #Write to parameter server for transparancy
        rospy.loginfo("[%s] %s = %s " %(self.node_name,param_name,value))
        return value


    def processSegments(self,segment_list_msg):
	propagateBelief()
        # initialize measurement likelihood
        measurement_likelihood = np.zeros(d.shape)
        for segment in segment_list_msg.segments:
            if segment.color != segment.WHITE and segment.color != segment.YELLOW:
                continue
            d_i,phi_i = self.generateVote(segment)
            if d_i > self.d_max or d_i < self.d_min or phi_i < self.phi_min or phi_i>self.phi_max:
                continue
            i = floor((d_i - self.d_min)/self.delta_d)
            j = floor((d_j - self.phi_min)/self.delta_phi)
            measurement_likelihood[i,j] += 1
        measurement_likelihood = measurement_likelihood/np.linalg.norm(measurement_likelihood)
        self.updateBelief(measurement_likelihood)
        # TODO entropy test:
        # TODO publish


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
        self.beliefRV=multiply(self.beliefRV,measurement_likelihood)
        self.beliefRV=beliefRV/np.linalg.norm(self.beliefRV)

    def generateVote(self,segment):
        p1 = segment.points[0]
        p2 = segment.points[1]
        d_i = 0.5*(p1.x+p2.x)
        if segment.color == segment.WHITE:
            d_i -= 0.5*self.lanewidth
            if p2.x > p1.x:
                d_i -= self.linewidth_white
        else: # yellow
            d_i += self.lanewidth
            if p2.x > p1.x:
                d_i += self.linewidth_yellow
        phi_i = pi/2 - atan2(abs(p2.x - p1.x), p2.y-p2.y)
        return d_i, phi_i
    
    def onShutdown(self):
        rospy.loginfo("[LaneFilterNode] Shutdown.")

if __name__ == '__main__': 
    rospy.init_node('lane_filter',anonymous=False)
    lane_filter_node = LaneFilterNode()
    rospy.on_shutdown(lane_filter_node.onShutdown)
    rospy.spin()

