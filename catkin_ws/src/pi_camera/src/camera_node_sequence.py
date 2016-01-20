#!/usr/bin/env python
import rospy
import cv2
import io
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image, CompressedImage, CameraInfo
from picamera import PiCamera
from picamera.array import PiRGBArray
import time
import signal
import sys
import rospkg
import os.path
import yaml

class CameraNode(object):
    def __init__(self):
        self.node_name = rospy.get_name()
        rospy.loginfo("[%s] Initializing......" %(self.node_name))

        self.framerate = self.setupParam("~framerate",60.0)
        self.res_w = self.setupParam("~res_w",320)
        self.res_h = self.setupParam("~res_h",200)
        self.decompress = self.setupParam("~decompress",False)
        self.cali_file_name = self.setupParam("~cali_file_name","default")

        rospack = rospkg.RosPack()
        self.cali_file = rospack.get_path('pi_camera') + "/calibration/" + self.cali_file_name + ".yaml" 
        self.camera_info_msg = None

        if not os.path.isfile(self.cali_file):
            rospy.logwarn("[%s] Can't find calibration file: %s.\nUsing default calibration instead." %(self.node_name,self.cali_file))
            self.cali_file = rospack.get_path('pi_camera') + "/calibration/default.yaml" 

        rospy.loginfo("[%s] Using calibration file: %s" %(self.node_name,self.cali_file))
        self.camera_info_msg = self.loadCameraInfo(self.cali_file)
        self.pub_camera_info = rospy.Publisher("~camera_info",CameraInfo,queue_size=1,latch=True)
        self.pub_camera_info.publish(self.camera_info_msg)

        self.has_published = False
        
        if self.decompress:
            self.pub_img= rospy.Publisher("~image/raw",Image,queue_size=1)
            self.bridge = CvBridge()
        else:
            self.pub_img= rospy.Publisher("~image/compressed",CompressedImage,queue_size=1)

        # Setup PiCamera
        self.stream = io.BytesIO()
        self.camera = PiCamera()
        self.camera.framerate = self.framerate
        self.camera.resolution = (self.res_w,self.res_h)

        self.is_shutdown = False
        # Setup timer
        self.gen = self.grabAndPublish(self.stream,self.pub_img)
        rospy.loginfo("[%s] Initialized." %(self.node_name))

    def startCapturing(self):
        rospy.loginfo("[%s] Start capturing." %(self.node_name))
        self.camera.capture_sequence(self.gen,'jpeg',use_video_port=True,splitter_port=0)
        self.camera.close()
        rospy.sleep(rospy.Duration.from_sec(2.0))
        rospy.loginfo("[%s] Capture Ended." %(self.node_name))

    def grabAndPublish(self,stream,publisher):
        while True: #TODO not being triggere correctly when shutting down.
            if self.is_shutdown:
                rospy.loginfo("[%s] Stopping stream...." %(self.node_name))
                # raise StopIteration
                return

            yield stream
            # Construct image_msg
            # Grab image from stream
            stream.seek(0)
            stream_data = stream.getvalue()
            stamp = rospy.Time.now()
            if not self.decompress:
                # Generate compressed image
                image_msg = CompressedImage()
                image_msg.format = "jpeg"
                image_msg.data = stream_data
            else:
                # Generate raw image
                image_msg = Image()
                data = np.fromstring(stream_data, dtype=np.uint8)  #320 by 240 90Hz
                image = cv2.imdecode(data, cv2.CV_LOAD_IMAGE_COLOR) # drop to about 60Hz
                image_msg = self.bridge.cv2_to_imgmsg(image) # drop to about 30hz..

            image_msg.header.stamp = stamp
            publisher.publish(image_msg)
            
            # Clear stream
            stream.seek(0)
            stream.truncate()

            if not self.has_published:
                rospy.loginfo("[%s] Published the first image." %(self.node_name))
                self.has_published = True

            rospy.sleep(rospy.Duration.from_sec(0.001))

    def setupParam(self,param_name,default_value):
        value = rospy.get_param(param_name,default_value)
        rospy.set_param(param_name,value) #Write to parameter server for transparancy
        rospy.loginfo("[%s] %s = %s " %(self.node_name,param_name,value))
        return value

    def onShutdown(self):
        rospy.loginfo("[%s] Closing camera." %(self.node_name))
        # self.camera.stop_recording(splitter_port=0)
        # rospy.sleep(rospy.Duration.from_sec(2.0))
        self.camera.close()
        rospy.sleep(rospy.Duration.from_sec(2.0))
        rospy.loginfo("[%s] Shutdown." %(self.node_name))

    def signal_handler(self, signal, frame):
        print "==== Ctrl-C Pressed ==== "
        self.is_shutdown = True
        
    def loadCameraInfo(self, filename):
        stream = file(filename, 'r')
        calib_data = yaml.load(stream)
        cam_info = CameraInfo()
        cam_info.width = calib_data['image_width']
        cam_info.height = calib_data['image_height']
        cam_info.K = calib_data['camera_matrix']['data']
        cam_info.D = calib_data['distortion_coefficients']['data']
        cam_info.R = calib_data['rectification_matrix']['data']
        cam_info.P = calib_data['projection_matrix']['data']
        cam_info.distortion_model = calib_data['distortion_model']
        return cam_info

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
    rospy.init_node('camera',anonymous=False,disable_signals=True)
    camera_node = CameraNode()
    signal.signal(signal.SIGINT, camera_node.signal_handler)
    rospy.on_shutdown(camera_node.onShutdown)
    camera_node.startCapturing()
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
