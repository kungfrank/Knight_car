#!/usr/bin/env python
import rospy
import tf
import numpy as np
from duckietown_msgs.msg import AprilTags, TagDetection, TagInfo
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker, MarkerArray

class apriltags_visualizer_node(object):
    def __init__(self):
        # Save the name of the node
        self.node_name = rospy.get_name()
        rospy.loginfo("[%s] Initialzing." %(self.node_name))

        self.info = TagInfo()
        # Get vehicle name from namespace
        self.veh_name = rospy.get_namespace().strip("/")
        rospy.loginfo("[%s] Vehicle name: %s" %(self.node_name,self.veh_name))

        # Setup publisher
        self.pub_markers = rospy.Publisher("~apriltags_markers",MarkerArray,queue_size=1)

        self.sign_types = {"StreetName": self.info.S_NAME,
            "TrafficSign": self.info.SIGN,
            "Light": self.info.LIGHT,
            "Localization": self.info.LOCALIZE,
            "Vehicle": self.info.VEHICLE}
        self.traffic_sign_types = {"stop": self.info.STOP, 
            "yield": self.info.YIELD, 
            "no-right-turn": self.info.NO_RIGHT_TURN, 
            "no-left-turn": self.info.NO_LEFT_TURN, 
            "oneway-right": self.info.ONEWAY_RIGHT, 
            "oneway-left": self.info.ONEWAY_LEFT, 
            "4-way-intersect": self.info.FOUR_WAY, 
            "right-T-intersect": self.info.RIGHT_T_INTERSECT, 
            "left-T-intersect": self.info.LEFT_T_INTERSECT,
            "T-intersection": self.info.T_INTERSECTION,
            "do-not-enter": self.info.DO_NOT_ENTER,
            "pedestrian": self.info.PEDESTRIAN,
            "t-light-ahead": self.info.T_LIGHT_AHEAD,
            "duck-crossing": self.info.DUCK_CROSSING}

        # Setup subscriber
        self.sub_apriltags = rospy.Subscriber("~apriltags_in", AprilTags, self.processAprilTags,queue_size=1)

        rospy.loginfo("[%s] Initialzed." %(self.node_name))
    
    def processAprilTags(self, apriltags_msg):
        marker_array = MarkerArray()
        
        for (i, detection) in enumerate(apriltags_msg.detections):
            tag_info = apriltags_msg.infos[i]
            tag_translation = apriltags_msg.detections[i].transform.translation
            
            marker = Marker()
            marker.header.frame_id = self.veh_name
            marker.ns = self.veh_name + "/apriltag"
            marker.id = i
            marker.action = Marker.ADD
            marker.lifetime = rospy.Duration.from_sec(3)
            marker.type = Marker.TEXT_VIEW_FACING
            
            #
            marker.pose.orientation.x = 0
            marker.pose.orientation.y = 0
            marker.pose.orientation.z = 0
            marker.pose.orientation.w = 1
            

            marker.pose.position.x = tag_translation.x
            marker.pose.position.y = tag_translation.y
            marker.pose.position.z = tag_translation.z
            marker.scale.x = 0.3
            marker.scale.y = 0.3
            marker.scale.z = 0.3

            # We want to give different color based on tag type
            # StreetName = green, TrafficSign = Red, Light = yellow, Localization = white, Vehicle=blue
            if tag_info.tag_type == self.info.S_NAME:
                marker.color.r = 0.0
                marker.color.g = 0.5
                marker.color.b = 0.0
                marker.color.a = 1.0
                marker.text = str(tag_info.id) + ": " + tag_info.street_name
            elif tag_info.tag_type == self.info.SIGN:
                marker.color.r = 0.5
                marker.color.g = 0.0
                marker.color.b = 0.0
                marker.color.a = 1.0
                for key, value in self.traffic_sign_types.iteritems():
                    if value == tag_info.traffic_sign_type:
                        marker.text = str(tag_info.id) + ": " + key
                        print marker.text
            elif tag_info.tag_type == self.info.LIGHT:
                marker.color.r = 0.5
                marker.color.g = 0.5
                marker.color.b = 0.0
                marker.color.a = 1.0
                marker.text = str(tag_info.id) + ": " + "Traffic Light"
            elif tag_info.tag_type == self.info.LOCALIZE:
                marker.color.r = 1
                marker.color.g = 1
                marker.color.b = 1
                marker.color.a = 1.0
                marker.text = str(tag_info.id) + ": " + "Localization"
            elif tag_info.tag_type == self.info.VEHICLE:
                marker.color.r = 0
                marker.color.g = 0
                marker.color.b = 0.5
                marker.color.a = 1.0
                marker.text = str(tag_info.id) + ": " + tag_info.vehicle_name
            marker_array.markers.append(marker)
            
        self.pub_markers.publish(marker_array)

    def on_shutdown(self):
        rospy.loginfo("[%s] Shutting down." %(self.node_name))

if __name__ == '__main__':
    # Initialize the node with rospy
    rospy.init_node('apriltags_visualizer_node', anonymous=False)

    # Create the NodeName object
    node = apriltags_visualizer_node()

    # Setup proper shutdown behavior 
    rospy.on_shutdown(node.on_shutdown)
    
    # Keep it spinning to keep the node alive
    rospy.spin()
