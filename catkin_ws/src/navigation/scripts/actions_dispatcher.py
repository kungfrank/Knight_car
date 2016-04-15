#!/usr/bin/env python

import sys
import rospy
from navigation.srv import *
from navigation.msg import SourceTargetNodes
from duckietown_msgs.msg import FSMState
from std_msgs.msg import Int16, String

#adding logic because FSM publishes our state at a high rate
#not just everytime the mode changes but multiple times in each mode
firstUpdate = True

actions = []
pub = 0
#state_data = FSMState()
#state_data.state = 'JOYSTICK_CONTROL'

def dispatcher(data):
    global actions
    global pub
    global firstUpdate
#    global state_data
#    state_data = data
    if firstUpdate == False and data.state != 'INTERSECTION_CONTROL': #state_data.INTERSECTION_CONTROL:
        firstUpdate = True

    if firstUpdate == True and data.state == 'INTERSECTION_CONTROL' and actions: #data.INTERSECTION_CONTROL and actions:
        rospy.sleep(2)
        action = actions.pop(0)
        print 'Dispatched:', action
        if action == 's':
            pub.publish(Int16(1))
        elif action == 'r':
            pub.publish(Int16(2))
        elif action == 'l':
            pub.publish(Int16(0))
        elif action == 'w':
            pub.publish(Int16(-1))    
    
        action_str = ''
        for letter in actions:
            action_str += letter

        pubList.publish(action_str)
        firstUpdate = False

def graph_search(data):
    print 'Requesting map for src: ', data.source_node, ' and target: ', data.target_node
    global actions
#    global state_data
    rospy.wait_for_service('graph_search')
    try:
        graph_search = rospy.ServiceProxy('graph_search', GraphSearch)
        resp = graph_search(data.source_node, data.target_node)
        actions = resp.actions
        if actions:
            # remove 'f' (follow line) from actions and add wait action in the end of queue
            actions = [x for x in actions if x != 'f']
            actions.append('w')
            print 'Actions to be executed:', actions
            action_str = ''
            for letter in actions:
                action_str += letter
            pubList.publish(action_str)
#            dispatcher(state_data)
        else:
            print 'Actions to be executed:', actions

    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

if __name__ == "__main__":
    rospy.init_node('action_dispatcher')
    rospy.Subscriber("~mode", FSMState, dispatcher, queue_size = 1)
    rospy.Subscriber("~plan_request", SourceTargetNodes, graph_search)
    pub = rospy.Publisher("~turn_type", Int16, queue_size=1, latch=True)
    pubList = rospy.Publisher("~turn_plan", String, queue_size=1, latch=True)
    
    rospy.spin()
