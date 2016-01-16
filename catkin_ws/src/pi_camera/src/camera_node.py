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

class CameraNode(object):
    def __init__(self):
        self.node_name = rospy.get_name()
        rospy.loginfo("[%s] Initializing......" %(self.node_name))
        # TODO: load parameters
        self.framerate_for_low = 90.0

        # TODO: load camera info yaml file
        self.pub_img_low = rospy.Publisher("~img_low/compressed",CompressedImage,queue_size=1)
        # self.pub_img_high= rospy.Publisher("~img_high/compressed",CompressedImage,queue_size=1)
        
        self.has_published = False

        # Setup PiCamera
        self.stream = io.BytesIO()
        self.bridge = CvBridge()
        self.camera = PiCamera()
        self.camera.framerate = self.framerate_for_low
        self.camera.resolution = (320,240)
        # TODO setup other parameters of the camera such as exposure and white balance etc

        # Setup timer
        # self.timer_img_low = rospy.Timer(rospy.Duration.from_sec(1.0/self.framerate_for_low),self.cbTimerLow)
        self.gen = self.grabAndPublish(self.stream,self.pub_img_low)
        rospy.loginfo("[%s] Initialized." %(self.node_name))

    def startCapturing(self):
        rospy.loginfo("[%s] Start capturing." %(self.node_name))
        self.camera.capture_sequence(self.gen,'jpeg',use_video_port=True)

    def cbTimerLow(self,event):
        self.grabAndPublish(self.stream,self.pub_img_low)
    
    def cbTimerHigh(self,event):
        self.grabAndPublish(self.stream,self.pub_img_high)

    def grabAndPublish(self,stream,publisher):
        while not rospy.is_shutdown(): #TODO not being triggere correctly when shutting down.
            yield stream
            # Construct image_msg
            image_msg = CompressedImage()
            image_msg.header.stamp = rospy.Time.now()
            image_msg.format = "jpeg"
            # Grab image from stream
            stream.seek(0)
            image_msg.data = stream.getvalue()
            # Publish 
            publisher.publish(image_msg)
            # Clear stream
            stream.seek(0)
            stream.truncate()

            if not self.has_published:
                rospy.loginfo("[%s] Published the first image." %(self.node_name))
                self.has_published = True

        self.camera.close()
        rospy.loginfo("[%s] Shutting down...." %(self.node_name))
        return

    def onShutdown():
        # self.camera.close()
        rospy.loginfo("[%s] Shutdown." %(self.node_name))

# def output(publisher):
#     stream = io.BytesIO()
#     # image_msg = Image()
#     image_msg = CompressedImage()
#     image_msg.format = "jpeg"

#     bridge = CvBridge()
#     # while not rospy.is_shutdown():
#     while True:
#         if rospy.is_shutdown():
#             break
#         yield stream
#         stream.seek(0)
#         stream_value = stream.getvalue()
#         # data = np.fromstring(stream_value, dtype=np.uint8)  #320 by 240 90Hz
#         # image = cv2.imdecode(data, 1) # drop to about 60Hz
#         # image_msg = bridge.cv2_to_imgmsg(image) # drop to about 30hz...
#         image_msg.data = stream_value
#         publisher.publish(image_msg)
#         # Clean up stream
#         stream.seek(0)
#         stream.truncate()
#     return

if __name__ == '__main__': 
    rospy.init_node('camera',anonymous=False)
    camera_node = CameraNode()
    camera_node.startCapturing()
    rospy.on_shutdown(camera_node.onShutdown)
    # pub_image = rospy.Publisher("~image/compressed",CompressedImage,queue_size=1)
    # pub_image = rospy.Publisher("~image",Image,queue_size=1)
    # resolution = (320,240)
    # resolution = (640,480)
    # # initialize the camera
    # with PiCamera() as camera:
    #     camera.resolution = resolution
    #     camera.framerate = 90
    #     gen = output(publisher=pub_image)
    #     # camera.capture_sequence(gen,'bgr',use_video_port=True)
    #     camera.capture_sequence(gen,'jpeg',use_video_port=True)
    rospy.spin()
