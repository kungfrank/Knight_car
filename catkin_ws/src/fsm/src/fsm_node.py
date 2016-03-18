#!/usr/bin/env python
import rospy
from duckietown_msgs.msg import FSMState, BoolStamped, StopLineReading, LanePose, CoordinationClearance
from std_msgs.msg import String #Imports msg

class FSMNode(object):
    def __init__(self):
        self.actual = FSMState()
        self.actual.state = FSMState.LANE_FOLLOWING
        self.actual.header.stamp = rospy.Time.now()
        self.in_lane = False
        self.at_stop_line = False
        self.intersection_go = False
        self.intersection_done = False

        # Save the name of the node
        self.node_name = rospy.get_name()
        
        rospy.loginfo("[%s] Initialzing." %(self.node_name))

        # Setup publishers
        self.pub_topic_mode = rospy.Publisher("~mode",FSMState, queue_size=1, latch=True)
        
        # Setup subscribers
        self.sub_topic_in_lane = rospy.Subscriber("~lane_pose", LanePose, self.cbInLane, queue_size=1)
        self.sub_topic_at_stop_line = rospy.Subscriber("~stop_line_reading", StopLineReading, self.cbAtStopLine, queue_size=1)
        self.sub_topic_intersection_go = rospy.Subscriber("~clearance_to_go", CoordinationClearance, self.cbIntersectionGo, queue_size=1)
        self.sub_topic_intersection_done = rospy.Subscriber("~intersection_done", BoolStamped, self.cbIntersectionDone, queue_size=1)

        # Read parameters
        rospy.loginfo("[%s] Initialzed." %(self.node_name))

    def cbIntersectionDone(self, done_msg):
        #print done_msg
        self.intersection_done = done_msg.data
        self.updateState(done_msg.header.stamp)

    def cbInLane(self, lane_pose_msg):
        #print lane_pose_msg
        self.in_lane = lane_pose_msg.in_lane
        self.updateState(lane_pose_msg.header.stamp)

    def cbAtStopLine(self, stop_line_reading_msg):
        #print stop_line_reading_msg
        self.at_stop_line = stop_line_reading_msg.at_stop_line
        self.updateState(stop_line_reading_msg.header.stamp)

    def cbIntersectionGo(self, go_msg):
        #print go_msg
        self.intersection_go = (go_msg.status == CoordinationClearance.GO)
        self.updateState(go_msg.header.stamp)

    def updateState(self,stamp):
        if(self.actual.state == self.actual.LANE_FOLLOWING):
            if(self.at_stop_line == True):
                #update the state
                self.actual.state = self.actual.COORDINATION
        elif(self.actual.state == self.actual.COORDINATION):
            if(self.intersection_go == True):
                self.actual.state = self.actual.INTERSECTION_CONTROL
                self.intersection_go = False
        elif(self.actual.state == self.actual.INTERSECTION_CONTROL):
            if(self.in_lane == True and self.intersection_done == True):
                self.actual.state = self.actual.LANE_FOLLOWING
                self.intersection_done = False

        self.actual.header.stamp = stamp
        self.pub_topic_mode.publish(self.actual)
   
    def setupParameter(self,param_name,default_value):
        value = rospy.get_param(param_name,default_value)
        rospy.set_param(param_name,value) #Write to parameter server for transparancy
        rospy.loginfo("[%s] %s = %s " %(self.node_name,param_name,value))
        return value

    def on_shutdown(self):
        rospy.loginfo("[%s] Shutting down." %(self.node_name))

if __name__ == '__main__':
    # Initialize the node with rospy
    rospy.init_node('fsm_node', anonymous=False)

    # Create the NodeName object
    node = FSMNode()

    # Setup proper shutdown behavior 
    rospy.on_shutdown(node.on_shutdown)
    # Keep it spinning to keep the node alive
    rospy.spin()
