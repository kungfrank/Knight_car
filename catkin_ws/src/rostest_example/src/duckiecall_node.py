#!/usr/bin/env python
import rospy
from rostest_example.Quacker import *
from std_msgs.msg import String, Int32

# Duckiecall Node
# Author: Teddy Ort
# Inputs: ~number_of_quacks/Int32 - The number of quacks that should be in the message
# Outputs: ~duckiecall/String - The output duckiecall message containing a series of quacks

class DuckiecallNode(object):
    def __init__(self):
        self.node_name = 'duckiecall_node'

        # Setup the publisher and subscriber
        self.sub_num_of_quacks = rospy.Subscriber("~number_of_quacks", Int32, self.quacksCallback)
        self.pub_duckiecall = rospy.Publisher("~duckiecall", String, queue_size=1)

        # Setup the quacker
        self.quacker = Quacker()

        rospy.loginfo("[%s] has started", self.node_name)

    def quacksCallback(self, msg_quacks):
        msg_duckiecall = String()
        msg_duckiecall.data = self.quacker.get_quack_string(msg_quacks.data)
        self.pub_duckiecall.publish(msg_duckiecall)


if __name__ == '__main__':
    rospy.init_node('duckiecall_node', anonymous=False)
    duckiecall_node = DuckiecallNode()
    rospy.spin()