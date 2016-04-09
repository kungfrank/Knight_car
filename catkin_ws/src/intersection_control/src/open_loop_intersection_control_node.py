#!/usr/bin/env python
import rospy
from duckietown_msgs.msg import FSMState, BoolStamped, WheelsCmdStamped
from std_msgs.msg import String, Int16 #Imports msg
import copy

class OpenLoopIntersectionNode(object):
    def __init__(self):
        # Save the name of the node
        self.node_name = rospy.get_name()
        self.mode = None
        self.turn_type = -1
        self.in_lane = False

        self.pub_cmd = rospy.Publisher("~wheels_cmd",WheelsCmdStamped,queue_size=1)
        self.pub_done = rospy.Publisher("~intersection_done",BoolStamped,queue_size=1)

        # Construct maneuvers
        self.maneuvers = dict()
        # forward_cmd = WheelsCmdStamped(vel_left=0.4,vel_right=0.4)
        # left_cmd = WheelsCmdStamped(vel_left=-0.25,vel_right=0.25)
        # right_cmd = WheelsCmdStamped(vel_left=0.25,vel_right=-0.25)
        # stop_cmd = WheelsCmdStamped(vel_left=0.0,vel_right=0.0)

        # turn_left = [(2.3,forward_cmd),(0.6,left_cmd),(2.0,forward_cmd)]
        # turn_right = [(2,forward_cmd),(0.6,right_cmd),(2.0,forward_cmd)]
        # turn_forward = [(3.0,forward_cmd),(0.0,forward_cmd),(3.0,forward_cmd)]
        # turn_stop = [(5.0,stop_cmd)]

        self.maneuvers[0] = self.getManeuver("turn_left")
        self.maneuvers[1] = self.getManeuver("turn_forward")
        self.maneuvers[2] = self.getManeuver("turn_right")
        # self.maneuvers[-1] = self.getManeuver("turn_stop")

        self.rate = rospy.Rate(30)

        # Subscribers
        self.sub_in_lane = rospy.Subscriber("~in_lane", BoolStamped, self.cbInLane, queue_size=1)
        self.sub_turn_type = rospy.Subscriber("~turn_type", Int16, self.cbTurnType, queue_size=1)
        self.sub_mode = rospy.Subscriber("~mode", FSMState, self.cbFSMState, queue_size=1)

    def getManeuver(self,param_name):
        param_list = rospy.get_param("~%s"%(param_name))
        # rospy.loginfo("PARAM_LIST:%s" %param_list)        
        maneuver = list()
        for param in param_list:
            maneuver.append((param[0],WheelsCmdStamped(vel_left=param[1],vel_right=param[2])))
        # rospy.loginfo("MANEUVER:%s" %maneuver)
        return maneuver

    def cbTurnType(self,msg):
        if self.mode == "INTERSECTION_CONTROL":
            self.turn_type = msg.data #Only listen if in INTERSECTION_CONTROL mode
            self.trigger(self.turn_type)

    def cbInLane(self,msg):
        self.in_lane = msg.data

    def cbFSMState(self,msg):
        if (not self.mode == "INTERSECTION_CONTROL") and msg.state == "INTERSECTION_CONTROL":
            # Switch into INTERSECTION_CONTROL mode
            rospy.loginfo("[%s] %s triggered." %(self.node_name,self.mode))
        self.mode = msg.state
        self.turn_type = -1 #Reset turn_type at mode change

    def publishDoneMsg(self):
        msg = BoolStamped()
        msg.header.stamp = rospy.Time.now()
        msg.data = True
        self.pub_done.publish(msg)
        rospy.loginfo("[%s] interesction_done!" %(self.node_name))
    
    def trigger(self,turn_type):
        if turn_type == -1: #Wait. Publish stop command. Does not publish done.
            cmd = WheelsCmdStamped(vel_left=0.0,vel_right=0.0)
            cmd.header.stamp = rospy.Time.now()
            self.pub_cmd.publish(cmd)
            return

        published_already = False
        for index, pair in enumerate(self.maneuvers[turn_type]):
            cmd = copy.deepcopy(pair[1])
            start_time = rospy.Time.now()
            end_time = start_time + rospy.Duration.from_sec(pair[0])
            while rospy.Time.now() < end_time:
                if not self.mode == "INTERSECTION_CONTROL": # If not in the mode anymore, return
                    return
                cmd.header.stamp = rospy.Time.now()
                self.pub_cmd.publish(cmd)
                if index > 1:
                    # See if need to publish interesction_done
                    if self.in_lane and not (published_already):
                        published_already = True
                        self.publishDoneMsg()
                        return
                self.rate.sleep()
        # Done with the sequence
        if not published_already:
            self.publishDoneMsg()

    def on_shutdown(self):
        rospy.loginfo("[%s] Shutting down." %(self.node_name))

if __name__ == '__main__':
    # Initialize the node with rospy
    rospy.init_node('open_loop_intersection_node', anonymous=False)

    # Create the NodeName object
    node = OpenLoopIntersectionNode()

    # Setup proper shutdown behavior 
    rospy.on_shutdown(node.on_shutdown)
    # Keep it spinning to keep the node alive
    rospy.spin()
