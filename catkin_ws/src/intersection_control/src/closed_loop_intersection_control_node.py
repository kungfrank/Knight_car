#!/usr/bin/env python
import rospy
from duckietown_msgs.msg import FSMState, BoolStamped, WheelsCmdStamped
from std_msgs.msg import String, Int16, Float32 #Imports msg
import copy

class OpenLoopIntersectionNode(object):
    def __init__(self):
        # Save the name of the node
        self.node_name = rospy.get_name()
        self.mode = None
        self.turn_type = -1
        self.in_lane = False
        self.ibvs_data = -1

        ibvs_topic = "/arii/ibvs"
        self.sub_ibvs = rospy.Subscriber(ibvs_topic, Float32, self.cbIbvs, queue_size=1)
        self.pub_cmd = rospy.Publisher("~wheels_cmd",WheelsCmdStamped,queue_size=1)
        self.pub_done = rospy.Publisher("~intersection_done",BoolStamped,queue_size=1)

        self.rate = rospy.Rate(30)

        # Subscribers
        self.sub_in_lane = rospy.Subscriber("~in_lane", BoolStamped, self.cbInLane, queue_size=1)
        self.sub_turn_type = rospy.Subscriber("~turn_type", Int16, self.cbTurnType, queue_size=1)
        self.sub_mode = rospy.Subscriber("~mode", FSMState, self.cbFSMState, queue_size=1)

    def cbTurnType(self,msg):
        self.turn_type = msg.data

    def cbIbvs (self,data):
        self.ibvs_data = data.data

    def cbInLane(self,msg):
        self.in_lane = msg.data

    def cbFSMState(self,msg):
        if (not self.mode == "INTERSECTION_CONTROL") and msg.state == "INTERSECTION_CONTROL":
            self.mode = msg.state
            rospy.loginfo("[%s] %s triggered. turn_type: %s" %(self.node_name,self.mode,self.turn_type))
            self.servo(self.turn_type)
        
        self.mode = msg.state


    def servo(self, turn_type):
        #move forward

        wheels_cmd_msg = WheelsCmdStamped()
        end_time = rospy.Time.now() + rospy.Duration(0.5)
        while rospy.Time.now() < end_time:
            wheels_cmd_msg.header.stamp = rospy.Time.now()
            wheels_cmd_msg.vel_left = .4 # go straight
            wheels_cmd_msg.vel_right = .4
            self.pub_cmd.publish(wheels_cmd_msg)

        wheels_cmd_msg = WheelsCmdStamped()
        wheels_cmd_msg.header.stamp = rospy.Time.now()
        while not rospy.is_shutdown():
            #self.mode == "INTERSECTION_CONTROL": # If not in the mode anymore, return
            angle_direction = (0.5 - self.ibvs_data)
            wheels_cmd_msg = WheelsCmdStamped()
            wheels_cmd_msg.header.stamp = rospy.Time.now()
            wheels_cmd_msg.vel_left = 0
            wheels_cmd_msg.vel_right = 0
            gain = 0.5
            done = False

            if abs(angle_direction) < 0.1 or self.ibvs_data == -1:
                if self.ibvs_data == -1:
                    rospy.loginfo("nothing detected!")
                    wheels_cmd_msg.vel_left = .2 # go straight
                    wheels_cmd_msg.vel_right = -.2
                else:
                    rospy.loginfo("already centered!")

                    done= True
                    wheels_cmd_msg.vel_left = .4 # go straight
                    wheels_cmd_msg.vel_right = .4
            else:
                if angle_direction > 0:
                    wheels_cmd_msg.vel_left = gain*abs(angle_direction)
                    rospy.loginfo("turning left %f " % angle_direction)
                else:
                    wheels_cmd_msg.vel_right = gain*abs(angle_direction)
                    rospy.loginfo("turning right %f " % angle_direction)
            self.pub_cmd.publish(wheels_cmd_msg)
            if self.in_lane:# and done==True:
                self.pub_done.publish(msg)

            self.rate.sleep()


    def publishDoneMsg(self):
        if self.mode == "INTERSECTION_CONTROL":
            msg = BoolStamped()
            msg.header.stamp = rospy.Time.now()
            msg.data = True
            self.pub_done.publish(msg)
            rospy.loginfo("[%s] interesction_done!" %(self.node_name))
    
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
