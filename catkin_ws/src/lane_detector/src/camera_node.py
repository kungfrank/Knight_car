#!/usr/bin/env python
import rospy
import cv2
import io
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from sensor_msgs.msg import CompressedImage
from picamera import PiCamera
from picamera.array import PiRGBArray
import time

def onShutdown():
    rospy.loginfo("[CameraNode] Shutdown.")

def output(publisher):
    stream = io.BytesIO()
    # image_msg = Image()
    image_msg = CompressedImage()
    image_msg.format = "jpeg"

    bridge = CvBridge()
    # while not rospy.is_shutdown():
    while True:
        if rospy.is_shutdown():
            break
        yield stream
        stream.seek(0)
        stream_value = stream.getvalue()
        # data = np.fromstring(stream_value, dtype=np.uint8)  #320 by 240 90Hz
        # image = cv2.imdecode(data, 1) # drop to about 60Hz
        # image_msg = bridge.cv2_to_imgmsg(image) # drop to about 30hz...
        image_msg.data = stream_value
        publisher.publish(image_msg)
        # Clean up stream
        stream.seek(0)
        stream.truncate()
    return

if __name__ == '__main__': 
    rospy.init_node('camera',anonymous=False)
    rospy.on_shutdown(onShutdown)
    pub_image = rospy.Publisher("~image/compressed",CompressedImage,queue_size=1)
    # pub_image = rospy.Publisher("~image",Image,queue_size=1)

    resolution = (320,240)
    # resolution = (640,480)
    # initialize the camera
    with PiCamera() as camera:
        camera.resolution = resolution
        camera.framerate = 90
        gen = output(publisher=pub_image)
        # camera.capture_sequence(gen,'bgr',use_video_port=True)
        camera.capture_sequence(gen,'jpeg',use_video_port=True)

    rospy.spin()
