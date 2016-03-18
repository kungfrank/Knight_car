#!/usr/bin/env python
import rospy
#from fsm.util import HelloGoodbye #Imports module. Not limited to modules in this pkg. 
from duckietown_msgs.msg import FSMState
from std_msgs.msg import String #Imports msg
from std_msgs.msg import Bool #Imports msg
#from duckietown_msgs.msg import messages to command the wheels
#from duckietown_msgs.msg import WheelsCmdStamped

class FSMNode(object):
    def __init__(self):
        self.actual = FSMState()
        self.actual.state = self.actual.LANE_FOLLOWING
        self.in_lane = False
        self.at_stop_line = False
        self.intersection_go = False
        self.intersection_done = False
        self.vehicle_detected = False

        # Save the name of the node
        self.node_name = rospy.get_name()
        
        rospy.loginfo("[%s] Initialzing." %(self.node_name))

        # Setup publishers
        self.pub_topic_mode = rospy.Publisher("~mode",FSMState, queue_size=1, latch=True)
        self.pub_topic_intersection_done = rospy.Publisher("~intersection_done",Bool, queue_size=1)
        self.pub_topic_intersection_go = rospy.Publisher("~intersection_go",Bool, queue_size=1)
        # Setup subscribers
        self.sub_topic_in_lane = rospy.Subscriber("~in_lane", Bool, self.cbInLane, queue_size=1)
        self.sub_topic_at_stop_line = rospy.Subscriber("~at_stop_line", Bool, self.cbAtStopLine, queue_size=1)
        self.sub_topic_intersection_go = rospy.Subscriber("~intersection_go", Bool, self.cbIntersectionGo, queue_size=1)
        self.sub_topic_intersection_done = rospy.Subscriber("~intersection_done", Bool, self.cbIntersectionDone, queue_size=1)
        self.sub_topic_vehicle_detected = rospy.Subscriber("~vehicle_detected",Bool,self.cbVehicleDetected, queue_size=1)

        # Read parameters
        self.pub_timestep = self.setupParameter("~pub_timestep",1.0)
        # Create a timer that calls the cbTimer function every 1.0 second
        #self.timer = rospy.Timer(rospy.Duration.from_sec(self.pub_timestep),self.cbTimer)

        rospy.loginfo("[%s] Initialzed." %(self.node_name))

        self.rate = rospy.Rate(30) # 10hz

    def cbVehicleDetected(self, in_lane_msg):
        print in_lane_msg
        self.vehicle_detected = in_lane_msg.data
        self.updateState()

    def cbIntersectionDone(self, in_lane_msg):
        print in_lane_msg
        self.intersection_done = in_lane_msg.data
        self.updateState()


    def cbInLane(self, in_lane_msg):
        print in_lane_msg
        self.in_lane = in_lane_msg.data
        self.updateState()

    def cbAtStopLine(self, in_lane_msg):
        print in_lane_msg
        self.at_stop_line = in_lane_msg.data
        self.updateState()

    def cbIntersectionGo(self, in_lane_msg):
        print in_lane_msg
        self.intersection_go = in_lane_msg.data
        self.updateState()


    def updateState(self):
        if(self.actual.state == self.actual.LANE_FOLLOWING):
            if(self.at_stop_line == True):
                self.actual.state = self.actual.COORDINATION
            elif(self.vehicle_detected == True):
                self.actual.state = self.actual.VEHICLE_AVOIDANCE
        elif(self.actual.state == self.actual.COORDINATION):
            if(self.intersection_go == True):
                self.actual.state = self.actual.INTERSECTION_CONTROL
                self.intersection_go = False
        elif(self.actual.state == self.actual.INTERSECTION_CONTROL):
            if(self.in_lane == True and self.intersection_done == True):
                self.actual.state = self.actual.LANE_FOLLOWING
                self.intersection_done = False
        elif(self.actual.state == self.actual.VEHICLE_AVOIDANCE):
            if(self.vehicle_detected == False and self.in_lane):
                self.actual.state = self.actual.LANE_FOLLOWING

        self.pub_topic_mode.publish(self.actual.state)


   
    def setupParameter(self,param_name,default_value):
        value = rospy.get_param(param_name,default_value)
        rospy.set_param(param_name,value) #Write to parameter server for transparancy
        rospy.loginfo("[%s] %s = %s " %(self.node_name,param_name,value))
        return value

    def cbTopic(self,msg):
        rospy.loginfo("[%s] %s" %(self.node_name,msg.data))

#    def cbTimer(self,event):
#        singer = HelloGoodbye()
        # Simulate hearing something
#        msg = String()
#        msg.data = "duckietown"
#singer.sing("duckietown")
#        self.pub_topic_a.publish(msg)
#        wheels_cmd_msg = WheelsCmdStamped()
#        wheels_cmd_msg.vel_left = 0.1
#        wheels_cmd_msg.vel_right = 0.1
#        self.pub_wheels_cmd.publish(wheels_cmd_msg)

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
