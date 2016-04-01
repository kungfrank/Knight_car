#!/usr/bin/env python
import rospy
from duckietown_msgs.msg import WheelsCmdStamped, BoolStamped, ObstacleProjectedDetectionList, ObstacleProjectedDetection

class MDOAPControllerNode:
    def __init__(self):
        self.name = 'mdoap_controller_node'
        rospy.loginfo('[%s] started', self.name)
        self.sub_close = rospy.Subscriber("~too_close", BoolStamped, self.cbBool, queue_size=1)
        self.sub_control = rospy.Subscriber("~lane_control", WheelsCmdStamped, self.cbLaneControl, queue_size=1)
        self.sub_detections = rospy.Subscriber("~detection_list_proj", ObstacleProjectedDetectionList, self.cbDetections, queue_size=1)
        self.pub_wheels_cmd = rospy.Publisher("~control",WheelsCmdStamped,queue_size=1)
        self.too_close = False
        self.lane_control = WheelsCmdStamped()
        self.lane_control.vel_left = 0.0
        self.lane_control.vel_right = 0.0

    def setupParameter(self,param_name,default_value):
        value = rospy.get_param(param_name,default_value)
        rospy.set_param(param_name,value) #Write to parameter server for transparancy
        rospy.loginfo("[%s] %s = %s " %(self.name,param_name,value))
        return value
    def cbLaneControl(self, lane_control_msg):
        self.lane_control = lane_control_msg
    def cbBool(self, bool_msg):
        self.too_close = bool_msg.data
    def cbDetections(self, detections_msg):
        if self.too_close:
            minDist = 999999999999999999999.0
            offset = 0.0
            # + -> offset to left in lane
            # - -> offset to right in lane
            for projected in detections_msg.list:
                # ~0.23 is the lane width
                if projected.distance < minDist and abs(projected.location.y) < 0.15:
                    minDist = projected.distance
                    y = projected.location.y
                    # + y -> obstacle to the left, drive right (set offset negative)
                    # - y -> obstacle to the right, drive left (set offset positive)
                    if y > 0:
                        offset = (-0.15 - y)*1.5
                    else:
                        offset = (0.15 - y)*1.5
            #Hijack the param for seting offset of the lane

            rospy.set_param("lane_controller_node/d_offset", offset)
        else:
            #Reset offset of lane to 0
            rospy.set_param("lane_controller_node/d_offset", 0.0)
        # stop = WheelsCmdStamped()
        # stop.header = bool_msg.header
        # stop.vel_left = 0.0
        # stop.vel_right = 0.0

        # Slow it down so it's easier to see what's going on for now
        self.lane_control.vel_left = self.lane_control.vel_left
        self.lane_control.vel_right = self.lane_control.vel_right
        self.pub_wheels_cmd.publish(self.lane_control)

if __name__=="__main__":
    rospy.init_node('mdoap_controller_node')
    node = MDOAPControllerNode()
    rospy.spin()