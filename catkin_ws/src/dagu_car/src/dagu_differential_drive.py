#!/usr/bin/env python
import rospy
from pkg_name.modulename import ModuleName
from duckietown_msgs.msg import CarControl

from dagu_car.daguddrive import DAGU_Differential_Drive

class DaguCar(object):
    def __init__(self):
        # Setup publishers
        self.dagu = DAGU_Differential_Drive()
        self.dagu.setSpeed(0.0)
        self.dagu.setSteerAngle(0.0)

        self.control_msg = CarControl()
        self.control_msg.speed = 0.0
        self.control_msg.steering = 0.0

        # Setup subscribers
        self.sub_topic = rospy.Subscriber("/car_supervisor/car_control", CarControl, self.cbControl)

        # Create a timer that calls the cbTimer function every 1.0 second
        self.timer = rospy.Timer(rospy.Duration.from_sec(0.02),self.cbTimer)
        rospy.loginfo("[DaguCar] Initialzed.")

    def cbControl(self,msg):
        self.control_msg = msg

    def cbTimer(self,event):
        self.dagu.setSpeed(self.control_msg.speed)
        self.dagu.setSteerAngle(self.control_msg.steering)

    def on_shutdown(self):
        rospy.loginfo("[DaguCar] Shutting down.")

if __name__ == '__main__':
    # Initialize the node with rospy
    rospy.init_node('dagu_car', anonymous=False)

    # Create the DaguCar object
    node = DaguCar()

    # Setup proper shutdown behavior 
    rospy.on_shutdown(node.on_shutdown)
    
    # Keep it spinning to keep the node alive
    rospy.spin()
