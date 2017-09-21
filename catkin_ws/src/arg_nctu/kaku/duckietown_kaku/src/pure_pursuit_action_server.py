#!/usr/bin/env python
import rospy
from geometry_msgs.msg import PoseStamped, PointStamped, Twist, Point, Pose, PoseArray
from std_msgs.msg import Bool
from gazebo_msgs.msg import ModelStates
from nav_msgs.msg import Path
import duckietown_kaku.msg
import actionlib
import numpy as np
import math
import tf

"""
This program utilizes pure pursuit to follow a given trajectory.
"""

class path_following_action():
    def __init__(self):
        # Init subscribers and publishers
        self.sub_model_state = rospy.Subscriber("/gazebo/model_states", ModelStates, self.model_stateCB, queue_size=1)
        self.pub_gazebo = rospy.Publisher('/duckiebot_with_gripper/cmd_vel', Twist, queue_size=1)

        self.PF_action = actionlib.SimpleActionServer("path_following_action", duckietown_kaku.msg.path_followingAction, execute_cb=None, auto_start = False)
        self.PF_action.register_goal_callback(self.actionCB)
        self.path_following_feedback = duckietown_kaku.msg.path_followingFeedback()
        self.path_following_result = duckietown_kaku.msg.path_followingResult()
        self.PF_action.start()
       
        # Init attributes
        self.default_speed = 0.1

        self.speed = self.default_speed
        self.steering_angle = 0

        self.robot_length = 0.22

        self.robot_pose = (0, 0, 0)#(-0.3, -0.1789, -0.0246)
        self.destination_pose = None
        self.threshold_proximity = 0.4      # How close the robot needs to be to the final waypoint to stop driving
        self.distance_from_path = None

        self.init_param()

    def init_param(self):

        self.waypoints = []#[(0.0565, 0.07184), (1.31215, 0.064377), (1.78436, 0.541745), (2.12162, 1.5687), (2.2123, 2.75912)]
        self.current_waypoint_index = 0
        self.lookahead_distance = 1.0

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

    def actionCB(self):
        self.init_param()
        goal = PoseArray()
        goal = self.PF_action.accept_new_goal()
        # print "action start, and points are",goal.poses
        for i in range(len(goal.waypoints.poses)):
            self.waypoints.append((goal.waypoints.poses[i].position.x,goal.waypoints.poses[i].position.y))
        print "waypoints are ",self.waypoints 

        rospy.loginfo('Path_following_action: Start')

    def model_stateCB(self, msg):
        if not self.PF_action.is_active():
            return 
        # print "start to path_following"
        quaternion_msg = [msg.pose[1].orientation.x, msg.pose[1].orientation.y, msg.pose[1].orientation.z, msg.pose[1].orientation.w]
        euler = tf.transformations.euler_from_quaternion(quaternion_msg)
        self.robot_pose = (msg.pose[1].position.x, msg.pose[1].position.y, euler[2])
        # print self.robot_pose
        # print 'robot_pose:', self.robot_pose#, 'waypoints:', self.waypoints
        self.destination_pose = self.circleIntersect(self.lookahead_distance)

        if self.destination_pose == None:
            print "approach destination  "
            self.gazebo_cmd(0,0)
            self.path_following_result.state = True
            rospy.loginfo('Path_following_action: Succeeded')
            self.PF_action.set_succeeded(self.path_following_result)

        else:
            # print 'destination_pose:', self.destination_pose
            # print 'waypoint_index:', self.current_waypoint_index    
            self.speed = self.default_speed
            distance_to_destination= self.getDistance(self.robot_pose, self.destination_pose)
            angle_to_destination = -self.getAngle(self.robot_pose, self.destination_pose)       
            # self.steering_angle = np.arctan((2 * self.robot_length * np.sin(angle_to_destination)) / distance_to_destination)         
            # print "robot_head",euler[2]*180/math.pi,"angle_to_destination", angle_to_destination*180/math.pi
            w = (angle_to_destination + math.pi) / (2 * math.pi) - 0.5
            self.gazebo_cmd(self.speed,w)

    def distanceBtwnPoints(self, x1, y1, x2, y2):
        return math.sqrt((x2 - x1)**2 + (y2 - y1)**2)

    def isPointOnLineSegment(self, x, y, x_start, y_start, x_end, y_end):
        return round(self.distanceBtwnPoints(x_start, y_start, x, y) + self.distanceBtwnPoints(x, y, x_end, y_end), 5) == round(self.distanceBtwnPoints(x_start, y_start, x_end, y_end), 5)

    # Find point on path that is closest to current robot position
    def closestPoint(self):
        x_robot, y_robot = self.robot_pose[:2]

        # Initialize values
        x_closest, y_closest = None, None
        waypoint_index = self.current_waypoint_index
        shortest_distance = self.distanceBtwnPoints(x_robot, y_robot, self.waypoints[waypoint_index][0], self.waypoints[waypoint_index][1]) #float('inf')

        for i in range(self.current_waypoint_index, len(self.waypoints) - 1):
            x1, y1 = self.waypoints[i]
            x2, y2 = self.waypoints[i+1]

            # For line (segment) equation ax + by + c = 0
            a = y1 - y2
            b = x2 - x1
            c = -b*y1 - a*x1  # Equivalently: x1*y2 - x2*y1

            x_close = (b*(b*x_robot - a*y_robot) - a*c)/(a**2 + b**2)
            y_close = (a*(-b*x_robot + a*y_robot) - b*c)/(a**2 + b**2)

            if not self.isPointOnLineSegment(x_close, y_close, x1, y1, x2, y2):
                continue

            distance = self.distanceBtwnPoints(x_robot, y_robot, x_close, y_close)

            if distance < shortest_distance:
                shortest_distance = distance
                x_closest = x_close
                y_closest = y_close
                waypoint_index = i

        if waypoint_index != self.current_waypoint_index:
            print "current waypoint_index is No.%d"%(waypoint_index)
            self.path_following_feedback.waypoint_index = waypoint_index
            self.PF_action.publish_feedback(self.path_following_feedback)
        self.current_waypoint_index = waypoint_index
        self.distance_from_path = shortest_distance
        return (x_closest, y_closest)

    # Find next point along the path at which the lookahead distance "circle" intersects, or None if we can stop driving
    def circleIntersect(self, lookahead_distance):
        if len(self.waypoints) == 0:
            return None
        if len(self.waypoints) == 1:
            x_endpoint, y_endpoint = self.waypoints[-1]
            x_robot, y_robot = self.robot_pose[:2]
            if self.distanceBtwnPoints(x_endpoint, y_endpoint, x_robot, y_robot) <= self.threshold_proximity:
                return None
            return self.waypoints[0]

        # If we are at the second-to-last waypoint and the robot is close enough to final waypoint, return None (to signal "stop")
        if self.current_waypoint_index == len(self.waypoints) - 2:
            x_endpoint, y_endpoint = self.waypoints[-1]
            x_robot, y_robot = self.robot_pose[:2]
            if self.distanceBtwnPoints(x_endpoint, y_endpoint, x_robot, y_robot) <= self.threshold_proximity:
                return None
            else:
                return self.waypoints[-1]

    # We only want to search the first path line segment past the point at which the robot is, so we make a "fake waypoint" for the current location of the robot along the path, and search from there onwards
        fake_robot_waypoint = self.closestPoint()
        if fake_robot_waypoint == (None, None):
            return self.waypoints[self.current_waypoint_index + 1]
        
        waypoints_to_search = [fake_robot_waypoint] + self.waypoints[self.current_waypoint_index + 1 : ]

        # If lookahead distance is shorter than distance from path, recall function with larger lookahead distance
        if lookahead_distance < self.distance_from_path:
            return self.circleIntersect(self.distance_from_path + 0.1)

        # For circle equation (x - p)^2 + (y - q)^2 = r^2
        p, q = self.robot_pose[:2]
        r = lookahead_distance

        # Check line segments along path until intersection point is found or we run out of waypoints
        for i in range(len(waypoints_to_search) - 1):
            # For line (segment) equation y = mx + b
            x1, y1 = waypoints_to_search[i]
            x2, y2 = waypoints_to_search[i+1]

            if x2 - x1 != 0:
                m = (y2 - y1)/(x2 - x1)
                b = y1 - m*x1

                #print "r=", r

                # Quadratic equation to solve for x-coordinate of intersection point
                A = m**2 + 1
                B = 2*(m*b - m*q - p)
                C = q**2 - r**2 + p**2 - 2*b*q + b**2

                if B**2 - 4*A*C < 0:    # Circle does not intersect line
                    continue
                
                # Points of intersection (could be the same if circle is tangent to line)
                x_intersect1 = (-B + math.sqrt(B**2 - 4*A*C))/(2*A)
                x_intersect2 = (-B - math.sqrt(B**2 - 4*A*C))/(2*A)
                y_intersect1 = m*x_intersect1 + b
                y_intersect2 = m*x_intersect2 + b
            else:
                x_intersect1 = x1
                x_intersect2 = x1
                y_intersect1 = q - math.sqrt(-x1**2 + 2*x1*p - p**2 + r**2)
                y_intersect2 = q + math.sqrt(-x1**2 + 2*x1*p - p**2 + r**2)

            # See if intersection points are on this specific segment of the line
            #print 'x_int, yint', x_intersect1, y_intersect1, x_intersect2, y_intersect2
            #print 'waypoints', x1, y1, x2, y2
            if self.isPointOnLineSegment(x_intersect1, y_intersect1, x1, y1, x2, y2):
                #rospy.loginfo('is returning')
                return (x_intersect1, y_intersect1)
            elif self.isPointOnLineSegment(x_intersect2, y_intersect2, x1, y1, x2, y2):
                #rospy.loginfo('is returning2')
                return (x_intersect2, y_intersect2)

        # If lookahead circle does not intersect the path at all (and the other two conditions at beginning of this function failed), then reduce the lookahead distance
        return self.circleIntersect(lookahead_distance - 0.1)

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
    rospy.init_node("path_following_action",anonymous=False)
    path_following_action = path_following_action()
    rospy.spin()


