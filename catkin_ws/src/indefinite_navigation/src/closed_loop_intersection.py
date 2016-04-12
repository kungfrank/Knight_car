#!/usr/bin/env python
import rospy
from intersection_control.util import HelloGoodbye #Imports module. Not limited to modules in this pkg. 
from duckietown_msgs.msg import LanePose, StopLineReading

from std_msgs.msg import String, Float32 #Imports msg
from std_msgs.msg import Bool #Imports msg
#from duckietown_msgs.msg import messages to command the wheels
from duckietown_msgs.msg import WheelsCmdStamped

class IndefNavigationNode(object):
    def __init__(self):
        # Save the name of the node
        self.node_name = rospy.get_name()
        
        rospy.loginfo("[%s] Initialzing." %(self.node_name))
	veh_name= rospy.get_param("veh")['duckiebot_visualizer']['veh_name']
	wheel_topic = veh_name + "/wheels_driver_node/wheels_cmd"
        lane_topic = veh_name + "/lane_filter_node/lane_pose"
        stop_topic = veh_name + "/stop_line_filter_node/stop_line_reading"
        ibvs_topic = "/arii/ibvs"

        self.lane = None
        self.stop = None
        self.ibvs_data = -1
        self.pub_wheels_cmd = rospy.Publisher(wheel_topic,WheelsCmdStamped, queue_size=1)
        self.sub_lane = rospy.Subscriber(lane_topic, LanePose, self.cbLane, queue_size=1) 
        self.sub_stop = rospy.Subscriber(stop_topic, StopLineReading, self.cbStop, queue_size=1) 
        
        self.sub_ibvs = rospy.Subscriber(ibvs_topic, Float32, self.cbIbvs, queue_size=1)

        rospy.loginfo("[%s] Initialzed." %(self.node_name))

        self.rate = rospy.Rate(3) # 10hz
    
    def cbIbvs (self,data):
        self.ibvs_data = data.data
    def cbLane(self, data):
        self.lane = data

    def cbStop(self, data):
        self.stop = data

    def servo(self):
        #move forward
        #end = rospy.Time.now() + rospy.Duration(.5)

        while not rospy.is_shutdown():

            spin_for = rospy.Time.now() + rospy.Duration(1.0)
            while not rospy.is_shutdown() and rospy.Time.now() < spin_for:            
                wheels_cmd_msg = WheelsCmdStamped()
                wheels_cmd_msg.header.stamp = rospy.Time.now()
                wheels_cmd_msg.vel_left = 1
                wheels_cmd_msg.vel_right = -1
                self.pub_wheels_cmd.publish(wheels_cmd_msg)    
            
            locate_for = rospy.Time.now() + rospy.Duration(7.0)
            while not rospy.is_shutdown() and rospy.Time.now() < locate_for:
                angle_direction = (self.ibvs_data - 0.5)
                wheels_cmd_msg = WheelsCmdStamped()
                wheels_cmd_msg.header.stamp = rospy.Time.now()
                wheels_cmd_msg.vel_left = 0
                wheels_cmd_msg.vel_right = 0
                gain = .1
                if abs(angle_direction) < 0.2 or self.ibvs_data == -1:
                    if self.ibvs_data == -1:
                        rospy.loginfo("nothing detected!")
                        
                        wheels_cmd_msg.vel_left  =-.1
                        wheels_cmd_msg.vel_right = -.1
                    else:
                        rospy.loginfo("already centered!")
                        wheels_cmd_msg.vel_left  =.4
                        wheels_cmd_msg.vel_right =.4
                else:
                    if angle_direction > 0:
                        wheels_cmd_msg.vel_left = gain*abs(angle_direction)
                        wheels_cmd_msg.vel_right = -gain *abs(angle_direction)
                        rospy.loginfo("turning left %f " % angle_direction)
                    else:
                        wheels_cmd_msg.vel_right = gain *abs(angle_direction)
                        wheels_cmd_msg.vel_left = -gain *abs(angle_direction)
                        rospy.loginfo("turning right %f " % angle_direction)
                self.pub_wheels_cmd.publish(wheels_cmd_msg)    
                self.rate.sleep()


if __name__ == '__main__':
    # Initialize the node with rospy
    rospy.init_node('indef_navigation_node', anonymous=False)

    # Create the NodeName object
    node = IndefNavigationNode()
    node.servo()

