#!/usr/bin/env python
import rospy
import smach
import smach_ros
from geometry_msgs.msg import Twist, Point, PoseArray, Pose
from gazebo_msgs.srv import GetModelState
from duckietown_kaku.msg import path_followingAction, path_followingGoal, gripper_modeAction, gripper_modeGoal, gripper_grabAction, gripper_grabGoal
from std_msgs.msg import Bool, Float32


class decide_next_object(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['finished'])
        self.index = 0
        print "================== now object ===================",self.index
        
    def execute(self, userdata):
        rospy.loginfo('DECIDE NEXT OBJECT')
        self.index += 1
        print "===================== index is %d =================" %self.index
                
        return 'finished'

def way_point1():
    waypoints = PoseArray()
    
    # waypoint_name = ["coke_can", "coke_can_0", "coke_can_1", "coke_can_2"]
    waypoint_name = ["my_cylinder", "my_cylinder_0", "my_cylinder_1", "my_cylinder_2", "Stop Sign"]

    for i in range(5):
    	new_points = Pose()
        rospy.wait_for_service('/gazebo/get_model_state')
        try:
            get_model_state = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
            model_state = get_model_state(waypoint_name[i],"")
            new_points.position.x = model_state.pose.position.x
            new_points.position.y = model_state.pose.position.y
            waypoints.poses.append(new_points)
            # print new_points
            # waypoints.poses.position.x.append(model_state.pose.position.x)
            # waypoints.poses.position.y.append(model_state.pose.position.y)
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e
        
    # print waypoints.poses
    return waypoints

def way_point2():
    waypoints = PoseArray()
    
    # waypoint_name = ["coke_can", "coke_can_0", "coke_can_1", "coke_can_2"]
    waypoint_name = ["my_cylinder_3", "my_cylinder_4", "my_cylinder_5"]

    for i in range(3):
    	new_points = Pose()
        rospy.wait_for_service('/gazebo/get_model_state')
        try:
            get_model_state = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
            model_state = get_model_state(waypoint_name[i],"")
            new_points.position.x = model_state.pose.position.x
            new_points.position.y = model_state.pose.position.y
            waypoints.poses.append(new_points)
            # print new_points
            # waypoints.poses.position.x.append(model_state.pose.position.x)
            # waypoints.poses.position.y.append(model_state.pose.position.y)
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e
        
    # print waypoints.poses
    return waypoints

def stop_point_get():
    stop_point = PoseArray()
    new_points = Pose()
    rospy.wait_for_service('/gazebo/get_model_state')
    try:
        get_model_state = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
        model_state = get_model_state("Stop Sign","")
        new_points.position.x = model_state.pose.position.x
        new_points.position.y = model_state.pose.position.y
        stop_point.poses.append(new_points)

    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

    return stop_point

def object_point_get():
    object_point = Point()
    rospy.wait_for_service('/gazebo/get_model_state')
    try:
        get_model_state = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
        model_state = get_model_state("my_cylinder_2","")
        object_point.x = model_state.pose.position.x
        object_point.y = model_state.pose.position.y

    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

    return object_point

def main():
	rospy.init_node("ros_smach_server")
	waypoints1 = way_point1()
	waypoints2 = way_point2()
	stop_points = stop_point_get()
	object_point = waypoints1.poses[0:3]
	object_index = decide_next_object()
	# object_point = object_point_get()
	# print waypoints.poses
	sm = smach.StateMachine(outcomes=['succeeded','aborted','preempted'])
	with sm:
	 # smach.StateMachine.add('PATH_FOLLOWING', smach_ros.SimpleActionState("path_following_action", path_followingAction, goal=path_followingGoal(waypoints=waypoints)),{'succeeded':'GRIPPER_MODE'})	 
	 
	 sm_sub_grab = smach.StateMachine(outcomes=['succeeded','aborted','preempted'])
	 with sm_sub_grab:
	 	smach.StateMachine.add('APPROACH_OBJECT',smach_ros.SimpleActionState("gripper_mode_action", gripper_modeAction, goal=gripper_modeGoal(object_point=object_point[object_index.index].position)),{'succeeded':'GRASP_OBJECT'})
	 	smach.StateMachine.add('GRASP_OBJECT',smach_ros.SimpleActionState("gripper_grab_action", gripper_grabAction, goal=gripper_grabGoal(grasping_state=True)))#,{'succeeded':'succeeded'})
	 smach.StateMachine.add('GRIPPER_MODE', sm_sub_grab,transitions={'succeeded':'GO_TO_DESTINATION'})

	 sm_sub_go_destination = smach.StateMachine(outcomes=['succeeded','aborted','preempted'])
	 with sm_sub_go_destination:
	 	smach.StateMachine.add('GO_TO_STOP_SIGN', smach_ros.SimpleActionState("path_following_action", path_followingAction, goal=path_followingGoal(waypoints=waypoints1)),{'succeeded':'DROP_OBJECT'})
	 	smach.StateMachine.add('DROP_OBJECT',smach_ros.SimpleActionState("gripper_grab_action", gripper_grabAction, goal=gripper_grabGoal(grasping_state=False)))#,{'succeeded':'succeeded'})
	 smach.StateMachine.add('GO_TO_DESTINATION', sm_sub_go_destination,transitions={'succeeded':'RESTART_AND_REDO'})

	 sm_sub_init = smach.StateMachine(outcomes=['succeeded','aborted','preempted'])
	 with sm_sub_init:
	 	smach.StateMachine.add('DECIDE_NEXT_OBJECT', decide_next_object(),{'finished':'GO_TO_CHECKPOINT'})
	 	smach.StateMachine.add('GO_TO_CHECKPOINT',smach_ros.SimpleActionState("path_following_action", path_followingAction, goal=path_followingGoal(waypoints=waypoints2)))#,{'succeeded':'succeeded'})
	 smach.StateMachine.add('RESTART_AND_REDO', sm_sub_init,transitions={'succeeded':'GRIPPER_MODE'})	 

	sis = smach_ros.IntrospectionServer('smach_server', sm, '/SM_ROOT')
	sis.start()
	sm.execute()
	rospy.on_shutdown(onShutdown)
	rospy.spin()
	sis.stop()

def onShutdown(self):
    rospy.loginfo("[ros_smach_server] Shutdown.")
if __name__=="__main__":
    main()
