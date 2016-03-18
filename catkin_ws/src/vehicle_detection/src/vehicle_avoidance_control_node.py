#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from duckietown_msgs.msg import WheelsCmdStamped, VehiclePose

class f6_controller:

    def __init__(self):
        self.node_name = rospy.get_name()

        self.wheels_cmd_pub = rospy.Publisher("~vehicle_avoidance_control",WheelsCmdStamped, queue_size = 1)
        self.vehicle_detected_pub = rospy.Publisher("~vehicle_detected",Bool, queue_size=1)
        self.subscriber = rospy.Subscriber("~vehicle_pose",VehiclePose, self.callback,  queue_size = 1)

    def callback(self,data):
        distance = data.p
        min_distance = 0.7 #don't want to approach any closer than this distance in meters
        vehicle_detected = False
        if distance < min_distance
            vehicle_detected = True
        self.vehicle_detected_pub.publish(vehicle_detected)
        self.publishCmd()

    def publishCmd(self): #maybe include a stamp one day
        wheels_cmd_msg = WheelsCmdStamped()
        #wheels_cmd_msg.header.stamp = stamp
        wheels_cmd_msg.vel_left = 0.0
        wheels_cmd_msg.vel_right = 0.0

        self.wheels_cmd_pub.publish(wheels_cmd_msg)


   
def f6_control_node():

    rospy.init_node('f6_control_node', anonymous=True)
    controller = f6_controller()

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print "Shutting down F6 controller"


if __name__ == '__main__':
    f6_control_node()