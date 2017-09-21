#!/usr/bin/env python
import rospy
from geometry_msgs.msg import PoseStamped, PointStamped, Twist, Point
from std_msgs.msg import Bool
from gazebo_msgs.msg import ModelStates
from gazebo_msgs.srv import GetModelState
from nav_msgs.msg import Path
import duckietown_kaku.msg
import actionlib
import numpy as np
import math
import tf
import time

"""
This program utilizes pure pursuit to follow a given trajectory.
"""

class gripper_mode_action_server():
    def __init__(self):
        # Init subscribers and publishers
        
         # Init attributes
        self.default_speed = 0.1

        self.speed = self.default_speed
        self.steering_angle = 0

        self.robot_length = 0.22

        self.robot_pose = (0, 0, 0)#(-0.3, -0.1789, -0.0246)
        self.destination_pose = None

        self.distance_from_path = None
        self.threshold_proximity = 0.25      # How close the robot needs to be to the final waypoint to stop driving
        self.active = False

        self.GM_action = actionlib.SimpleActionServer("gripper_mode_action", duckietown_kaku.msg.gripper_modeAction, execute_cb=None, auto_start = False)
        self.GM_action.register_goal_callback(self.action_modeCB)
        self.gripper_mode_feedback = duckietown_kaku.msg.gripper_modeFeedback()
        self.gripper_mode_result = duckietown_kaku.msg.gripper_modeResult()
        self.GM_action.start()

        self.GG_action = actionlib.SimpleActionServer("gripper_grab_action", duckietown_kaku.msg.gripper_grabAction, execute_cb=self.action_grabCB, auto_start = False)
        self.gripper_grab_feedback = duckietown_kaku.msg.gripper_grabFeedback()
        self.gripper_grab_result = duckietown_kaku.msg.gripper_grabResult()
        self.GG_action.start()

        self.sub_model_state = rospy.Subscriber("/gazebo/model_states", ModelStates, self.model_stateCB, queue_size=1)
        self.pub_gazebo = rospy.Publisher('/duckiebot_with_gripper/cmd_vel', Twist, queue_size=1)
        self.pub_gazebo_gripper = rospy.Publisher('/duckiebot_gripper/cmd_vel', Twist, queue_size=1)

    def action_grabCB(self, goal):
        if goal.grasping_state == True:
            self.gripper_cmd(0,0.5)
            time.sleep(5) 
            print "grasp object"
        else:
            self.gripper_cmd(0,-0.5)
            time.sleep(5) 
            print "drop object"

        rospy.loginfo('gripper_grab_action: Succeeded')
        self.GG_action.set_succeeded()

    def action_modeCB(self):
        goal = Point()
        goal = self.GM_action.accept_new_goal()
        # print "action start, and points are",goal.poses
        self.destination_pose = (goal.object_point.x,goal.object_point.y)
        print "++++++++++++++++++++++++   ",self.destination_pose,"  ++++++++++++++++++++++++++"

    def gazebo_cmd(self, v, w):
        model_state_msg = Twist()
        model_state_msg.linear.x = v
        model_state_msg.linear.y = 0
        model_state_msg.linear.z = 0

        model_state_msg.angular.x = 0
        model_state_msg.angular.y = 0
        model_state_msg.angular.z = -w
        
        self.pub_gazebo.publish(model_state_msg)
        print "publish cmd from pure_pursuit_action"


    def gripper_cmd(self, v, w):
        model_state_msg = Twist()
        model_state_msg.linear.x = v
        model_state_msg.linear.y = 0
        model_state_msg.linear.z = 0

        model_state_msg.angular.x = 0
        model_state_msg.angular.y = 0
        model_state_msg.angular.z = -w
        
        self.pub_gazebo_gripper.publish(model_state_msg)

    def model_stateCB(self, msg):
        if not self.GM_action.is_active():
            return 
        # print self.destination_pose
        quaternion_msg = [msg.pose[1].orientation.x, msg.pose[1].orientation.y, msg.pose[1].orientation.z, msg.pose[1].orientation.w]
        euler = tf.transformations.euler_from_quaternion(quaternion_msg)
        self.robot_pose = (msg.pose[1].position.x, msg.pose[1].position.y, euler[2])
            
        distance_to_destination= self.getDistance(self.robot_pose, self.destination_pose)
        angle_to_destination = -self.getAngle(self.robot_pose, self.destination_pose)

        if angle_to_destination < 3 * (math.pi/180):
            # print "correct straight line" 
            if distance_to_destination<= self.threshold_proximity:
                print "ready to grab"
                self.gazebo_cmd(0,0)

                self.gripper_mode_result.state = True
                rospy.loginfo('gripper_mode_action: Succeeded')
                self.GM_action.set_succeeded(self.gripper_mode_result)
            else:
                self.gazebo_cmd(self.speed,0)
        else:
            # print "correct orientation"    
            w = (angle_to_destination + math.pi) / (2 * math.pi) - 0.5
            self.gazebo_cmd(0,w)

        self.gripper_mode_feedback.distance = distance_to_destination
        self.GM_action.publish_feedback(self.gripper_mode_feedback)

    def getDistance(self, start_pose, end_pose):
        #Takes a starting coordinate (x,y,theta) and ending coordinate (x,y) and returns distance between them in map units
        delta_x = end_pose[0] - start_pose[0]
        delta_y = end_pose[1] - start_pose[1]

        distance = np.sqrt([delta_x**2 + delta_y**2])
        return distance[0]

    def getAngle(self, start_pose, end_pose):
        #Takes a starting coordinate (x,y,theta) and ending coordinate (x,y) and returns angle between them relative to the front of the car in degrees
        delta_x = end_pose[0] - start_pose[0]
        delta_y = end_pose[1] - start_pose[1]

        #rad_to_deg_conv = 180.0 / np.pi
        theta = start_pose[2] #* rad_to_deg_conv
     
        between_angle = np.arctan2(delta_y, delta_x) #* rad_to_deg_conv
        # print "between_angle", between_angle*180/math.pi
        return theta - between_angle



if __name__=="__main__":
    # Tell ROS that we're making a new node.
    rospy.init_node("gripper_mode_action_server",anonymous=False)
    gripper_mode_action_server = gripper_mode_action_server()
    rospy.spin()


