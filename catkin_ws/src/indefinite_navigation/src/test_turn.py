#!/usr/bin/env python
import rospy
import math
from intersection_control.util import HelloGoodbye #Imports module. Not limited to modules in this pkg. 
from duckietown_msgs.msg import LanePose, StopLineReading
from std_srvs.srv import Empty
from std_msgs.msg import String #Imports msg
from std_msgs.msg import Bool #Imports msg
#from duckietown_msgs.msg import messages to command the wheels
from duckietown_msgs.msg import Twist2DStamped, BoolStamped

class IndefNavigatioTurnNode(object):
    def __init__(self):
        # Save the name of the node
        self.node_name = rospy.get_name()
        
        rospy.loginfo("[%s] Initialzing." %(self.node_name))
        veh_name= rospy.get_param("veh")['duckiebot_visualizer']['veh_name']
        lane_topic = "/" + veh_name + "/lane_filter_node/lane_pose"
        done_topic = "/" + veh_name + "/open_loop_intersection_control_node/intersection_done"
        left_service = "/" + veh_name + "/open_loop_intersection_control_node/turn_left"
        right_service = "/" + veh_name + "/open_loop_intersection_control_node/turn_right"

        self.lane = None
        self.done = None

        self.sub_lane = rospy.Subscriber(lane_topic, LanePose, self.cbLane, queue_size=1)
        self.sub_done = rospy.Subscriber(done_topic, BoolStamped, self.cbDone, queue_size=1)
        rospy.loginfo("[%s] Initialzed." %(self.node_name))

        rospy.wait_for_service(left_service)
        self.turn_left_serv = rospy.ServiceProxy(left_service, Empty)
        
        rospy.wait_for_service(right_service)
        self.turn_right_serv = rospy.ServiceProxy(right_service, Empty)
        
        self.rate = rospy.Rate(30) # 10hz

    def cbLane(self, data):
        self.lane = data

    def cbDone(self, data):
        self.done = data.data

    def turn(self, type):
        if type.lower() == 'left':
            self.turn_left_serv(Empty)
        else:
            self.turn_right_serv(Empty)
        while not self.done:
            pass

        self.final = self.lane
        self.calculate()

    def calculate(self):
        init_d = 0.0
        init_phi = 0.0

        final_d = self.final.d
        final_phi = self.final.phi

        off_d = abs(init_d - final_d)
        off_phi = abs(init_phi - final_phi)
        result_trim = "FAILED"

        if abs(off_d) < 0.08:
            result_trim = "PASSED"

        info = """
        TURN RESULT
        ===================
        Goal location is (%.4f, %.4f), 
        final location is (%.4f, %.4f).
        
        distance offset = %.4f
        distance angle offset = %.4f
        TURN TEST % s
        """ % ( init_d, init_phi, final_d, final_phi, \
                off_d, off_phi, result_trim)
        print info




if __name__ == '__main__':
    # Initialize the node with rospy
    rospy.init_node('indef_navigation_turn_node', anonymous=False)

    # Create the NodeName object
    node = IndefNavigationTurnNode()
    turn_type = raw_input("left or right?")
    node.turn(turn_type)
    
    turn_type = raw_input("left or right?")
    node.turn(turn_type)

