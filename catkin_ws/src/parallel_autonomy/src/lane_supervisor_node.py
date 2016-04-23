#!/usr/bin/env python
import rospy
import numpy as np
import math
from std_msgs.msg import Bool
from duckietown_msgs.msg import Twist2DStamped, LanePose, StopLineReading
from sensor_msgs.msg import Joy


class lane_supervisor(object):
    def __init__(self):
        self.node_name = rospy.get_name()
        self.lane_reading = LanePose()
        self.car_control_lane = Twist2DStamped()
        self.car_control_joy = Twist2DStamped()
        self.safe = True
        # TODO: properly encode states in state machine
        self.state = 0
        self.in_lane = True
        self.at_stop_line = False
        self.stop = False

        # Params:
        self.max_cross_track_error = self.setupParameter("~max_cross_track_error", 0.1)
        self.max_heading_error = self.setupParameter("~max_heading_error", math.pi / 4)
        self.min_speed = self.setupParameter("~min_speed", 0.1)
        self.max_speed = self.setupParameter("~max_speed", 0.3)
        self.max_steer = self.setupParameter("~max_steer", 0.2)

        # Publication
        self.pub_car_cmd = rospy.Publisher("~car_cmd", Twist2DStamped, queue_size=1)
        self.pub_safe = rospy.Publisher("~safe", Bool, queue_size=1)

        # Subscriptions
        self.sub_lane_pose = rospy.Subscriber("~lane_pose", LanePose, self.cbLanePose, queue_size=1)
        self.sub_lane_control = rospy.Subscriber("~car_cmd_lane", Twist2DStamped, self.cbLaneControl, queue_size=1)
        self.sub_joy_control = rospy.Subscriber("~car_cmd_joy", Twist2DStamped, self.cbJoyControl, queue_size=1)
        self.sub_at_stop_line = rospy.Subscriber("~stop_line_reading", StopLineReading, self.cbStopLine, queue_size=1)

        self.params_update = rospy.Timer(rospy.Duration.from_sec(1.0), self.updateParams)

    def setupParameter(self, param_name, default_value):
        value = rospy.get_param(param_name, default_value)
        rospy.set_param(param_name, value)  # Write to parameter server for transparancy
        rospy.loginfo("[%s] %s = %s " % (self.node_name, param_name, value))
        return value

    def updateParams(self, event):
        self.max_cross_track_error = rospy.get_param("~max_cross_track_error")
        self.max_heading_error = rospy.get_param("~max_heading_error")
        self.min_speed = rospy.get_param("~min_speed")
        self.max_speed = rospy.get_param("~max_speed")
        self.max_steer = rospy.get_param("~max_steer")

    def cbStopLine(self, stop_line_msg):
        if not stop_line_msg.at_stop_line:
            self.at_stop_line = False
            self.stop = False
        else:
            if not self.at_stop_line:
                self.at_stop_line = True
                self.stop = True
                rospy.sleep(2)
                self.stop = False

    def cbLanePose(self, lane_pose_msg):
        self.in_lane = lane_pose_msg.in_lane
        self.lane_reading = lane_pose_msg
        cross_track_err = math.fabs(lane_pose_msg.d)
        heading_err = math.fabs(lane_pose_msg.phi)
        if cross_track_err > self.max_cross_track_error or heading_err > self.max_heading_error:
            self.safe = False
        else:
            self.safe = True
        self.pub_safe.publish(self.safe)

    def cbLaneControl(self, lane_control_msg):
        self.car_control_lane = lane_control_msg

    def cbJoyControl(self, joy_control_msg):
        self.car_control_joy = joy_control_msg
        car_cmd_msg = self.mergeJoyAndLaneControl()
        self.pub_car_cmd.publish(car_cmd_msg)

    def mergeJoyAndLaneControl(self):
        car_cmd_msg = Twist2DStamped()
        if self.stop:
            if self.state != 1:
                rospy.loginfo("[PA] stopped at stop line")
                self.state = 1
            car_cmd_msg.v = 0
            car_cmd_msg.omega = 0
        elif self.safe:  # or not self.in_lane:
            if self.state != 2:
                rospy.loginfo("[PA] in safe mode")
                self.state = 2
            self.car_control_joy.v = min(self.car_control_joy.v, self.max_speed)
            self.car_control_joy.omega = np.clip(self.car_control_joy.omega, -self.max_steer, self.max_steer)
            if self.car_control_joy.v < self.min_speed:
                # sets the speeds to 0:
                # TODO: reformat
                self.car_control_joy.v = 0.0
                self.car_control_joy.omega = 0.0
            car_cmd_msg = self.car_control_joy
            car_cmd_msg.header.stamp = self.car_control_joy.header.stamp
        else:
            if self.state != 3:
                rospy.loginfo("[PA] not safe - merge control inputs")
                self.state = 3
            car_control_merged = Twist2DStamped()
            if self.car_control_joy.v < self.min_speed:
                # sets the speeds to 0:
                car_control_merged.v = 0.0
                car_control_merged.omega = 0.0
            else:
                # take the speed from the joystick:
                car_control_merged.v = min(self.car_control_joy.v, self.max_speed)
                # take the omega from the lane controller:
                car_control_merged.omega = self.car_control_lane.omega
            car_cmd_msg = car_control_merged
            car_cmd_msg.header.stamp = self.car_control_joy.header.stamp
        return car_cmd_msg


if __name__ == "__main__":
    rospy.init_node("lane_supervisor", anonymous=False)
    lane_supervisor_node = lane_supervisor()
    rospy.spin()
