#!/usr/bin/env python
import rospy
import smach
import smach_ros
from geometry_msgs.msg import Twist, Point
from std_msgs.msg import Bool


class bar(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['bar_succeeded'])
    def execute(self, userdata):
        rospy.sleep(3.0)
        return 'bar_succeeded'
 
def gripper_cb(ud, msg):
    return False
def grab_cb(ud, msg):
    return False
 
def main():
	rospy.init_node("monitor_example")

	sm = smach.StateMachine(outcomes=['DONE'])
	with sm:
	 smach.StateMachine.add('PATH_FOLLOWING', smach_ros.MonitorState("/gripper_mode/object", Point, gripper_cb), transitions={'invalid':'READY_TO_GRAB', 'valid':'PATH_FOLLOWING', 'preempted':'PATH_FOLLOWING'})
	 smach.StateMachine.add('READY_TO_GRAB',smach_ros.MonitorState("/gripper_mode/grab_object", Bool, grab_cb), transitions={'invalid':'GO_TO_STOP_SIGN', 'valid':'READY_TO_GRAB', 'preempted':'READY_TO_GRAB'})
	 smach.StateMachine.add('GO_TO_STOP_SIGN',smach_ros.MonitorState("/gripper_mode/finished", Bool, grab_cb), transitions={'invalid':'DONE', 'valid':'GO_TO_STOP_SIGN', 'preempted':'GO_TO_STOP_SIGN'})

	sis = smach_ros.IntrospectionServer('smach_server', sm, '/SM_ROOT')
	sis.start()
	sm.execute()
	rospy.on_shutdown(onShutdown)
	rospy.spin()
	sis.stop()

def onShutdown(self):
    rospy.loginfo("[dc_grab] Shutdown.")
if __name__=="__main__":
    main()

# class dc_grab(object):
#     def __init__(self):
# 	    self.node_name = "dc_grab"
# 	    self.active = True
# 	    self.motorhat = Adafruit_MotorHAT(addr=0x60)
# 	    self.dc_grab = self.motorhat.getMotor(3)

# 	    self.sub_joy_ = rospy.Subscriber("joy", Joy, self.cbJoy, queue_size=1)
# 	    self.pub_gazebo_grab = rospy.Publisher('/duckiebot_with_gripper/gripper_cmd_vel', Twist, queue_size=1)

#     def cbJoy(self, msg):
# 		self.joy = msg
# 		self.processButtons(msg)

#     def processButtons(self, msg):
# 		grab_state_msg = Twist()
# 		grab_state_msg.linear.x = 0
# 		grab_state_msg.linear.y = 0
# 		grab_state_msg.linear.z = 0

# 		grab_state_msg.angular.x = 0
# 		grab_state_msg.angular.y = 0
# 		grab_state_msg.angular.z = 0

# 		if (self.joy.buttons[0] == 1):

# 			self.dc_grab.setSpeed(200)
# 			self.dc_grab.run(Adafruit_MotorHAT.BACKWARD)

# 			grab_state_msg.angular.z = -1
# 			self.pub_gazebo_grab.publish(grab_state_msg)

# 		if (self.joy.buttons[1] == 1):

# 			self.dc_grab.setSpeed(200)
# 			self.dc_grab.run(Adafruit_MotorHAT.FORWARD)

# 			grab_state_msg.angular.z = +1
# 			self.pub_gazebo_grab.publish(grab_state_msg)

# 		if (self.joy.buttons[2] == 1):

# 			self.dc_grab.run(Adafruit_MotorHAT.RELEASE)

 
#     def onShutdown(self):
#         rospy.loginfo("[dc_grab] Shutdown.")

# if __name__ == '__main__': 
#     rospy.init_node('dc_grab',anonymous=False)
#     dc_grab = dc_grab()
#     rospy.on_shutdown(dc_grab.onShutdown)
#     rospy.spin()

