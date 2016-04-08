#!/usr/bin/env python
import rospy
from intersection_control.util import HelloGoodbye #Imports module. Not limited to modules in this pkg. 
from duckietown_msgs.msg import LanePose, StopLineReading

from std_msgs.msg import String #Imports msg
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

        self.lane = None
        self.stop = None
        self.forward_time = 3.0

        self.pub_wheels_cmd = rospy.Publisher(wheel_topic,WheelsCmdStamped, queue_size=1)
        self.sub_lane = rospy.Subscriber(lane_topic, LanePose, self.cbLane, queue_size=1) 
        self.sub_stop = rospy.Subscriber(stop_topic, StopLineReading, self.cbStop, queue_size=1) 

        rospy.loginfo("[%s] Initialzed." %(self.node_name))

        self.rate = rospy.Rate(30) # 10hz

    def cbLane(self, data):
        self.lane = data

    def cbStop(self, data):
        self.stop = data

    def driveForward(self):
        #move forward
        for i in range(3):
            if self.lane == None or self.stop == None:
                rospy.loginfo("still waiting for lane and stop line")
                rospy.sleep(1)
        if self.lane==None or self.stop == None:
            rospy.loginfo("could not subscribe to lane and stop line")
            return
        
        self.init = self.lane, self.stop
        forward_for_time = self.forward_time
        starting_time = rospy.Time.now()
        while((rospy.Time.now() - starting_time) < rospy.Duration(forward_for_time)):
            wheels_cmd_msg = WheelsCmdStamped()
            wheels_cmd_msg.header.stamp = rospy.Time.now()
            wheels_cmd_msg.vel_left = 0.4
            wheels_cmd_msg.vel_right = 0.4
            self.pub_wheels_cmd.publish(wheels_cmd_msg)    
            #rospy.loginfo("Moving?.")
            self.rate.sleep()
        self.final = self.lane, self.stop
        self.calculate()

    def calculate(self):
        
        init_d = self.init[0].d
        init_phi = self.init[0].phi

        final_d = self.final[0].d
        final_phi = self.final[0].phi

        off_d = abs(init_d - final_d)
        off_phi = abs(init_phi - final_phi)

        result_trim = "PASSED/NOT PASSED "

        init_stop_y = self.init[1].stop_line_point.y
        final_stop_y = self.final[1].stop_line_point.y

        velocity = abs(init_stop_y - final_stop_y) / self.forward_time
        result_vel = "PASSED/NOT PASSED"
        
        info = """
        LANE OFFSET SUMMARY
        ===================
        initial location is (%.2f, %.2f), 
        final location is (%.2f, %.2f).

        distance offset = %.2f
        distance angle offset = %.2f
        TRIM TEST % s


        VELOCITY OFFSET SUMMARY
        ======================
        initial stop sign y offset: %.2f
        final stop sign y offset: %.2f
        velocity computed: %.2f
        VELOCITY TEST: %s

        """ % ( init_d, init_phi, final_d, final_phi, \
                off_d, off_phi, result_trim,\
                init_stop_y, final_stop_y, velocity, result_vel
                )
        print info




if __name__ == '__main__':
    # Initialize the node with rospy
    rospy.init_node('indef_navigation_node', anonymous=False)

    # Create the NodeName object
    node = IndefNavigationNode()
    raw_input("drive forward?")
    node.driveForward()

