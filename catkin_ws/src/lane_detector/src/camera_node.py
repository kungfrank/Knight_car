#!/usr/bin/env python
import rospy
import cv2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from picamera import PiCamera
from picamera.array import PiRGBArray
import time

def onShutdown():
    rospy.loginfo("[CameraNode] Shutdown.")

if __name__ == '__main__': 
    rospy.init_node('camera',anonymous=False)
    rospy.on_shutdown(onShutdown)
    pub_image = rospy.Publisher("~image",Image,queue_size=1)

    bridge = CvBridge()
    
    resolution = (320,240)
    # fps = 90
    # r = rospy.Rate(fps) # 10hz

    # initialize the camera
    with PiCamera() as camera:
        camera.resolution = resolution
        camera.framerate = 30
        rawCapture = PiRGBArray(camera, size=resolution)
        # # capture frames from the camera
        for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
            if rospy.is_shutdown():
                break
            image_msg = bridge.cv2_to_imgmsg(rawCapture.array, "bgr8")
            pub_image.publish(image_msg)
            rawCapture.truncate(0)
    rospy.spin()
