#!/usr/bin/env python
import rospy
from ground_projection.srv import *
from duckietown_msgs.msg import Vector2D, ObstacleImageDetection, ObstacleImageDetectionList, ObstacleType, Rect
from visualization_msgs.msg import MarkerArray
from visualization_msgs.msg import Marker

class ObstacleSafetyNode:
	def __init__(self):
		self.name = 'obstacle_safety_node'
		rospy.loginfo('[%s] started', self.name)

        self.sub_ = rospy.Subscriber("~object_image_detection_list", ObstacleImageDetectionList, self.cbDetectionsList, queue_size=1)

        # Uncomment for ground projection and markers
        rospy.wait_for_service('ground_projection/get_ground_coordinate')
        self.ground_proj = rospy.ServiceProxy('ground_projection/get_ground_coordinate',GetGroundCoord)

        #TODO(CL): Publish boolean once BooleanStamped from dev branch makes it in
        #self.pub_too_close = rospy.Publisher("~obstacle_markers", MarkerArray, queue_size=1)
        
        self.pub_markers = rospy.Publisher("~obstacle_markers", MarkerArray, queue_size=1)
        self.maxMarkers = 0
    def cbDetectionsList(self, detections_msg):
        #For ground projection uncomment the next lines

        marker_array = MarkerArray()

        p = Vector2D()
        count = 0;
        for obstacle in detections_msg.list: 
            marker = Marker()
            rect = obstacle.bounding_box
            p.x = float(rect.x)/float(rect.w)
            p.y = float(rect.y)/float(rect.h)
            projected_point = self.ground_proj(p)
            #print projected_point.gp
            marker.header = detections_msg.header
            marker.type = marker.ARROW
            marker.action = marker.ADD
            marker.scale.x = 0.01
            marker.scale.y = 0.01
            marker.scale.z = 0.1
            marker.color.a = 1.0
            marker.color.r = 1.0
            marker.color.g = 0.5
            marker.color.b = 0.0
            marker.pose.orientation.w = 1.0
            marker.pose.position.x = projected_point.gp.x
            marker.pose.position.y = projected_point.gp.y
            marker.pose.position.z = projected_point.gp.z 
            marker.id = count
            count = count +1

            marker_array.markers.append(marker)
       
        if count > self.maxMarkers:
            self.maxMarkers = count
            
        while count <= self.maxMarkers:
            marker = Marker()
            marker.action = 2
            marker.id = count
            marker_array.markers.append(marker)
            count = count+1

        self.pub_markers.publish(marker_array)

if __name__=="__main__":
	rospy.init_node('obstacle_safety_node')
	node = ObstacleSafetyNode()
	rospy.spin()