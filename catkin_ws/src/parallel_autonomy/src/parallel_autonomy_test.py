#!/usr/bin/env python
import rospy
import numpy as np
import math
from std_msgs.msg import Bool
from duckietown_msgs.msg import FSMState
from sensor_msgs.msg import Joy
from std_msgs.msg import String

class parallel_autonomy_test(object):
    def __init__(self):
        self.desired_supervisor_output = 'safe'
        self.passed = 0
        self.failed = 0
        # Save the name of the node
        self.node_name = rospy.get_name()


        rospy.loginfo("[%s] Initialzing." %(self.node_name))
        veh_name= rospy.get_param("~veh")
        # Publicaiton
        #self.result = rospy.Publisher("~result",Float64,queue_size=1)

        # Setup subscribers
        self.sub_FSM = rospy.Subscriber("/" + veh_name + "/mode", FSMState, self.cbMode, queue_size=1)
        self.sub_annotation = rospy.Subscriber("/" + veh_name + "/desired_supervisor_output",String, self.updateDesiredOutput, queue_size=1)

    def updateDesiredOutput(self,msg):
        self.desired_supervisor_output = msg.data

    def cbMode(self, msg):
        if(self.fsm_mode == "INTERSECTION_CONTROL" and self.desired_supervisor_output == 'intersection'):
        	self.passed +=1
        else:
        	self.failed +=1
        print self.passed/(float(self.passed+self.failed))




if __name__ == "__main__":
    rospy.init_node("parallel_autonomy_test_node",anonymous=False)
    parallel_autonomy_test_node = parallel_autonomy_test()
    rospy.spin()
