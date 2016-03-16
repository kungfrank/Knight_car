#!/usr/bin/env python

import rospy
import cv2
from sensor_msgs.msg import CompressedImage, Image
from std_msgs.msg import String, Header
from duckietown_msg_hanssusilo.msg import FlipDir
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import time
from os.path import expanduser


# Define callback function
def imgCB(msg):
        global imgnum
        imgcv =  bridge.imgmsg_to_cv2(msg, "bgr8")
        img_name = filedir+'0'+str(imgnum)
        flip = rospy.get_param("/penguin/virtual_mirror_hanssusilo_node/flip_direction")
        file_name = img_name+"_"+flip+".png"
        corrimg = cv2.imread(expanduser(file_name),cv2.IMREAD_COLOR)
        #print 'file_name:=' + file_name
        corrimgmsg = bridge.cv2_to_imgmsg(corrimg, "bgr8")
        h = Header()
        h.stamp = rospy.Time.now()
        corrimgmsg.header = h
        img_cor_pub.publish(corrimgmsg)
        if np.array_equal(imgcv,corrimg):
                out = "Success! Image is correct!"
        else:
                out = "Failed. Image is incorrect"
        img_stat_pub.publish(out)

def paramTimerCB(event):
        global imgnum
        flip = rospy.get_param("/penguin/virtual_mirror_hanssusilo_node/flip_direction")
        if flip == "vert":
                imgnum = imgnum+1
                if imgnum > 4:
                        imgnum = 1
        if flip == 'vert':
                flip = rospy.set_param("/penguin/virtual_mirror_hanssusilo_node/flip_direction",'horz')
                flip = 'horz'
        elif flip == "horz":
                flip = rospy.set_param("/penguin/virtual_mirror_hanssusilo_node/flip_direction",'vert')
                flip = 'vert'
        time.sleep(1.5)
        img_name = filedir+'0'+str(imgnum)
        file_name = img_name+"_orig.png"
        #print file_name
        origimg = cv2.imread(expanduser(file_name),cv2.IMREAD_COLOR)
        imgout = bridge.cv2_to_imgmsg(origimg, "bgr8")
        h = Header()
        h.stamp = rospy.Time.now()
        imgout.header = h
        img_org_pub.publish(imgout)
        
        

# Initialize the node with rospy
rospy.init_node('virtual_mirror_hanssusilo_tester_node')
flip = rospy.set_param("/penguin/virtual_mirror_hanssusilo_node/flip_direction",'vert')
filedir = rospy.get_param("/penguin/virtual_mirror_hanssusilo_node/test_directory")
imgnum = 0
bridge = CvBridge()
img_org_pub = rospy.Publisher("img_orig", Image,queue_size=1)
img_stat_pub = rospy.Publisher("img_status",String,queue_size=1)
img_cor_pub = rospy.Publisher("img_correct",Image,queue_size=1)
flip_img_sub = rospy.Subscriber("img_flipped", Image, imgCB)
param_timer = rospy.Timer(rospy.Duration.from_sec(4),paramTimerCB)
rospy.spin() #Keeps the script for exiting

