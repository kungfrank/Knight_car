#!/usr/bin/env python

import sys
import rospy
from navigation.srv import *
from duckietown_msgs.msg import FSMState
from std_msgs.msg import Int16

actions = []
pub = 0

def callback(data):
    global actions
    global pub
    if data.state == data.COORDINATION:
        action = actions.pop(0)
        if action == 's':
            pub.publish(Int16(1))
        elif action == 'r':
            pub.publish(Int16(0))
        elif action == 'l':
            pub.publish(Int16(2))        

def graph_search_client():
    rospy.wait_for_service('graph_search')
    try:
        graph_search = rospy.ServiceProxy('graph_search', GraphSearch)
        resp = graph_search('I15', 'I26')
        return resp.actions
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

if __name__ == "__main__":
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber("fsm/mode", FSMState, callback)
    pub = rospy.Publisher("/turn_type", Int16, queue_size=1)

    actions = graph_search_client()
    
    # remove 'f' (follow line) from actions
    actions = [x for x in actions if x != 'f']
    
    # while actions is not empty dispatch the next action
    while True:
        if len(actions) == 0:
            break
    # wait
    pub.publish(Int16(-1))
