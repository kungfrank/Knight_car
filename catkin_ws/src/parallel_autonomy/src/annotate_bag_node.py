#!/usr/bin/env python
import rospy
import rosbag
from std_msgs.msg import String
from os.path import basename

class AnnotateBagNode(object):
    def __init__(self):
        self.node_name = 'annotate_bag_node'
        rospy.loginfo("[%s] has started", self.node_name)
        self.topicin   = self.setupParam("~topicin", 'camera_node/image/compressed')
        self.topicout = self.setupParam("~topicout", 'desired_supervisor_behavior')
        self.bagin     = self.setupParam("~bagin", False)
        self.bagout    = self.setupParam("~bagout", self.bagin+".annotated.bag")
        self.annotation_filename = self.bagin[:-4]+"-annotation.txt"
        rospy.loginfo(self.annotation_filename)
        self.current_annotation = 'safe'
        self.processAnnotationFile()
        self.processBag()
        rospy.loginfo("[%s] Bag processing complete" %(self.node_name))

    def processBag(self):
        with rosbag.Bag(self.bagout, 'w') as outbag:
            for topic, msg, t in rosbag.Bag(self.bagin).read_messages():
                # rospy.loginfo("topic: %s, self.topicin: %s." %(topic,self.topicin))
                outbag.write(topic,msg,t)
                if rospy.is_shutdown():
                    break
                if topic == self.topicin: # annotate based on this topic's seq number
                    if msg.header.seq in self.annotation_dict:
                        self.current_annotation = self.annotation_dict[msg.header.seq]
                    annotation = self.current_annotation
                    msgout = String()
                    msgout.data = annotation
                    outbag.write(self.topicout, msgout, t)

    def processAnnotationFile(self):
        self.annotation_dict = {}
        with open(self.annotation_filename, 'r') as f:
            for line in f:
                line = line.strip() # remove \n at end
                l = line.split(',')
                self.annotation_dict[int(l[0][5:])] = l[1][6:]


    def setupParam(self,param_name,default_value):
        value = rospy.get_param(param_name,default_value)
        rospy.set_param(param_name,value) #Write to parameter server for transparancy
        rospy.loginfo("[%s] %s = %s " %(self.node_name,param_name,value))
        return value

if __name__ == '__main__':
    rospy.init_node('annotate_bag_node', anonymous=False)
    bag_stamper_node = AnnotateBagNode()