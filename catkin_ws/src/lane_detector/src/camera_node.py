#!/usr/bin/env python
import rospy
import cv2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from picamera.array import PiRGBArray
from picamera import PiCamera

def onShutdown():
    rospy.loginfo("[CameraNode] Shutdown.")

if __name__ == '__main__': 
    rospy.init_node('camera',anonymous=False)
    rospy.on_shutdown(line_detector_node.onShutdown)
    pub_image = rospy.Publisher("~image",Image,queue_size=1)

    bridge = CvBridge()
    
    resolution = (640, 480)
    fps = 60
    r = rospy.Rate(fps) # 10hz

    # initialize the camera
    camera = PiCamera()
    camera.resolution = resolution
    camera.framerate = fps
    rawCapture = PiRGBArray(camera, size=resolution)

    # capture frames from the camera
    # camera_capture_continous = camera.capture_continuous(rawCapture, format="bgr", use_video_port=True)

    for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
        if rospy.is_shutdown():
            break
        bgr = frame.array
        image_msg = self.bridge.cv2_to_imgmsg(bgr, "bgr8")
        pub_image.publish(image_msg)
        r.sleep()
 
    #   # show the frame
    #   cv2.imshow("Frame", bgr)
    #   cv2.waitKey(1)
 
    #   # clear the stream in preparation for the next frame
    #   rawCapture.truncate(0)
    rospy.spin()
