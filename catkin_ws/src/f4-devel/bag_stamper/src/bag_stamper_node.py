#!/usr/bin/env python
import rospy
import rosbag
from duckietown_msgs.msg import WheelsCmdStamped

class BagStamperNode(object):
    def __init__(self):
        self.node_name = 'bag_stamper_node'
        rospy.loginfo("[%s] has started", self.node_name)
        self.topicin   = self.setupParam("~topicin", 'wheels_driver/wheels_cmd')
        self.topicout = self.setupParam("~topicout", 'wheels_driver_node/wheels_cmd')
        self.bagin     = self.setupParam("~bagin", False)
        self.bagout    = self.setupParam("~bagout", self.bagin+".stamped.bag")
        count = self.processBag()
        rospy.loginfo("[%s] %s messages stamped. Bag processing complete", self.node_name, count)

    def processBag(self):
        count = 0
        with rosbag.Bag(self.bagout, 'w') as outbag:
            for topic, msg, t in rosbag.Bag(self.bagin).read_messages():
                if rospy.is_shutdown():
                    break
                if topic == self.topicin:
                    msgout = WheelsCmdStamped()
                    msgout.vel_left = msg.vel_left
                    msgout.vel_right = msg.vel_right
                    msgout.header.stamp = t
                    outbag.write(self.topicout, msgout, t)
                    count += 1
                else:
                    outbag.write(topic, msg, t)
        return count

    def setupParam(self,param_name,default_value):
        value = rospy.get_param(param_name,default_value)
        rospy.set_param(param_name,value) #Write to parameter server for transparancy
        rospy.loginfo("[%s] %s = %s " %(self.node_name,param_name,value))
        return value

if __name__ == '__main__':
    rospy.init_node('bag_stamper', anonymous=False)
    bag_stamper_node = BagStamperNode()