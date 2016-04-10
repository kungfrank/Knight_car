#!/usr/bin/env python
import rospy
from std_msgs.msg import Float32MultiArray, Int32
from rostest_example.Quacker import *

# Average Quacks Node
# Author: Teddy Ort
# Inputs: ~list/Float32MultiArray - A list of quacks to average
# Outputs: ~number_of_quacks/Int32 - The rounded average of the list received

class AverageQuacksNode(object):
    def __init__(self):
        self.node_name = 'average_quacks_node'
        rospy.loginfo("[%s] has started", self.node_name)

        # Setup the publisher and subscriber
        self.sub_list = rospy.Subscriber("~list", Float32MultiArray, self.listCallback)
        self.pub_quacks = rospy.Publisher("~number_of_quacks", Int32, queue_size=1)

        # Setup the quacker
        self.quacker = Quacker()

    def listCallback(self, msg):
        msg_num_of_quacks = Int32()
        msg_num_of_quacks.data = self.quacker.rounded_mean(msg.data)
        self.pub_quacks.publish(msg_num_of_quacks)

if __name__ == '__main__':
    rospy.init_node('average_quacks_node', anonymous=False)
    average_quacks_node = AverageQuacksNode()
    rospy.spin()