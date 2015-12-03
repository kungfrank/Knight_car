#!/usr/bin/python

# Low level controller node for the DC motors.
# Reads the /steer and /speed topics
#
# date:    11/17/2015
#
# authors: Valerio Varricchio <valerio@mit.edu>
#          Luca Carlone <lcarlone@mit.edu>
#
#!/usr/bin/env python

import rospy
import DCMotorInterface 
from std_msgs.msg import Float64

class motorInterfaceNode():

    def __init__(self):

        # Initialize DC_motor controller
        self.DCMI = DCMotorInterface.DCMotorInterface();
        self.speedTimestamp = 0;

        rospy.Subscriber("/speed", Float64, self.speedCallback)

    def speedCallback(self, data):
        #self.speedTimestamp = rospy.Time.now()

        self.DCMI.setSpeed(data.data)
        #rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
            

if __name__ == '__main__':
    
    # Init ROS network
    rospy.init_node('motorsController')
    node = motorInterfaceNode()
    rospy.spin()