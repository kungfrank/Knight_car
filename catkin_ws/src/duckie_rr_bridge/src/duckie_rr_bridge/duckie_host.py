#!/usr/bin/env python
import sys
import argparse
import numpy as np
import math
import time
import thread, threading
import string
import struct
import traceback
from __builtin__ import True, False

import rospy
from duckietown_msgs.msg import Twist2DStamped, BoolStamped, Pose2DStamped, LanePose, AprilTagDetectionArray
#from sensor_msgs.msg import CompressedImage
from sensor_msgs.msg import Image, CameraInfo

import RobotRaconteur as RR

duckie_servicedef="""
#Service to provide simple interface to the Duckiebot
service Duckiebot_Interface

option version 0.5

struct DuckieImage
    #field string format
    field int32 width
    field int32 height
    field int32 step
    field uint8[] data
end struct

struct ImageHeader
    #field string format
    field int32 width
    field int32 height
    field int32 step
end struct

struct LanePose
    field double d
    field double phi
    field double sigma_d
    field double sigma_phi
    field uint8 status
    field uint8 in_lane
end struct

struct AprilTag
    field double size
    field double[3] pos
    field double[4] quat 
end struct

object Duckiebot
    
    property double v
    property double omega
    property double x
    property double y
    property double theta
    property uint8 camera_open
    property LanePose lane_pose
    property AprilTag{list} april_tags

    event TagDetected(AprilTag at)

    function void sendCmd(double v, double omega)
    function void sendStop()
    function void openCamera()
    function void closeCamera()
    function DuckieImage getImage()
    function ImageHeader getImageHeader()
    #function LanePose getLanePose()

    pipe DuckieImage ImageStream

end object
"""

class DuckiebotHost(object):
    def __init__(self):
        rospy.init_node("duckie_rr_bridge",anonymous=False)
        self.node_name = rospy.get_name()
        rospy.loginfo("[%s] Initializing "%(self.node_name))

        self.last_pub_msg = None
        self.last_pub_time = rospy.Time.now()

        # Lock for multithreding 
        self._lock = threading.RLock()

        # for image pipe
        self._imagestream=None
        self._imagestream_endpoints = dict()
        self._imagestream_endpoints_lock = threading.RLock()

        # start with the camera closed
        self._camera_open = False

        # set constant ImageHeader structure
        # TODO: These shouldn't be hard coded...
        self._image_header = RR.RobotRaconteurNode.s.NewStructure("Duckiebot_Interface.ImageHeader")
        self._image_header.width = int(640)
        self._image_header.height = int(480)
        self._image_header.step = int(4)
        #self._image_header.format = "jpeg"

        # set DuckieImage struct
        self._image = RR.RobotRaconteurNode.s.NewStructure(
            "Duckiebot_Interface.DuckieImage")
        self._image.width = int(640)
        self._image.height = int(480)
        self._image.step = int(4)
        #self._image.format = "jpeg"


        # Setup Kinematic Properties
        self._v = 0
        self._omega = 0
        self._x = 0
        self._y = 0
        self._theta = 0

        # Set up Lane Pose Struct
        self._lane_pose = RR.RobotRaconteurNode.s.NewStructure(
            "Duckiebot_Interface.LanePose")
        self._lane_pose.d = float(0)
        self._lane_pose.phi = float(0)
        self._lane_pose.sigma_d = float(0)
        self._lane_pose.sigma_phi = float(0)
        self._lane_pose.status = int(0)
        self._lane_pose.in_lane = 0
        
        # Set up april tag struct
        self._apriltags = []
        self.TagDetected = RR.EventHook()

        # Publications
        self.pub_car_cmd = rospy.Publisher("~car_cmd", Twist2DStamped, queue_size=1)

        # Subscriptions 
        self.sub_velocity = rospy.Subscriber("~velocity", Twist2DStamped, self._cb_velocity)
        self.sub_pose = rospy.Subscriber("~pose",Pose2DStamped, self._cb_pose)
        self.sub_lane_pose = rospy.Subscriber("~lane_pose", LanePose, self._cb_lane_pose)
        self.sub_april = rospy.Subscriber("~april", AprilTagDetectionArray, self._cb_april)
    
    # Camera Functions
    @property
    def camera_open(self):
        if self._camera_open:
            return 1
        else:
            return 0

    def openCamera(self):
        if self._camera_open:
            return
        #start camera subscriptin
        rospy.loginfo(" Subscribing to %s"%(rospy.resolve_name("~image")) )
        #self.sub_image = rospy.Subscriber("~image",CompressedImage,self._cb_image, queue_size=1)
        self.sub_image = rospy.Subscriber("~image", Image, self._cb_image, queue_size=1)
        self._camera_open = True

    def closeCamera(self):
        if not self._camera_open:
            return

        if self.sub_image:
            self.sub_image.unregister()
        self._camera_open = False

    def _cb_image(self,imagedata):
        with self._lock:
            if imagedata.data:
                self._image.data = np.frombuffer(imagedata.data,dtype="u1")
                self._image.height = imagedata.height
                self._image.width = imagedata.width
                self._image.step = imagedata.step

        with self._imagestream_endpoints_lock:
            # send to pipe endpoints
            for ep in self._imagestream_endpoints:
                dict_ep = self._imagestream_endpoints[ep]
                # Loop through indices in nested dict
                for ind in dict_ep:
                    # Attempt to send frame to connected endpoint
                    try:
                        pipe_ep = dict_ep[ind]
                        pipe_ep.SendPacket(self._image)
                    except:
                        # on error, assume pipe has been closed
                        self._ImageStream_pipeclosed(pipe_ep)

    def getImage(self):
        with self._lock:
            return self._image

    def getImageHeader(self):
        return self._image_header

    # Image Pipe Functions
    @property
    def ImageStream(self):
        return self._imagestream
    
    @ImageStream.setter
    def ImageStream(self,value):
        self._imagestream = value
        # Set the PipeConnectCallback to ImageStream_pipeconnect
        # that will be called when a PipeEndpoint connects
        value.PipeConnectCallback = self._ImageStream_pipeconnect

    def _ImageStream_pipeconnect(self, pipe_ep):
        # lock the _imagestream_endpoints dictionary and place the pipe_ep in
        # a nested dict that is indexed by the endpoint of the client and the 
        # index of the pipe
        with self._imagestream_endpoints_lock:
            # if there is not an entry for this client endpoint, add it
            if (not pipe_ep.Endpoint in self._imagestream_endpoints):
                self._imagestream_endpoints[pipe_ep.Endpoint] = dict()

            # Add pipe_ep to the correct dictionary given the endpoint + index
            dict_ep = self._imagestream_endpoints[pipe_ep.Endpoint]
            dict_ep[pipe_ep.Index] = pipe_ep
            pipe_ep.PipeEndpointClosedCallback = self._ImageStream_pipeclosed
        
    def _ImageStream_pipeclosed(self, pipe_ep):
        with self._imagestream_endpoints_lock:
            try:
                dict_ep = self._imagestream_endpoints[pipe_ep.Endpoint]
                del(dict_ep[pipe_ep.Index])
            except:
                traceback.print_exc()

    # Lane Pose Functions
    @property
    def lane_pose(self):
        #with self._lock:
        return self._lane_pose

    def _cb_lane_pose(self,lane_pose_msg):
        self._lane_pose.d = lane_pose_msg.d
        self._lane_pose.phi = lane_pose_msg.phi
        self._lane_pose.sigma_d = lane_pose_msg.sigma_d
        self._lane_pose.sigma_phi = lane_pose_msg.sigma_phi
        self._lane_pose.status = lane_pose_msg.status
        if lane_pose_msg.in_lane:
            self._lane_pose.in_lane = 1
        else:
            self._lane_pose.in_lane = 0

    
    # April Tags Properties
    @property
    def april_tags(self):
        #with self._lock:
        return self._apriltags

    def _cb_april(self, april_msg):
        self._apriltags = []

        for tag in april_msg.detections:
            apriltag = RR.RobotRaconteurNode.s.NewStructure(
            "Duckiebot_Interface.AprilTag") 
            
            apriltag.size = tag.size
            pose = tag.pose.pose
            pos = pose.position
            ori = pose.orientation
            
            apriltag.pos = np.array([pos.x, pos.y, pos.z], dtype='f8')
            apriltag.quat = np.array([ori.x, ori.y, ori.z, ori.w], dtype='f8')
            
            self.TagDetected.fire(apriltag)
            self._apriltags.append(apriltag)

    # Kinematics Functions
    @property
    def v(self):
        return self._v
    
    @property
    def omega(self):
        return self._omega
    
    @property
    def x(self):
        return self._x
    
    @property
    def y(self):
        return self._y
    
    @property
    def theta(self):
        return self._theta
    
    def sendCmd(self, v, omega):
        car_cmd_msg = Twist2DStamped()
        car_cmd_msg.header.stamp = rospy.Time.now()
        car_cmd_msg.v = v
        car_cmd_msg.omega = omega

        self.pub_car_cmd.publish(car_cmd_msg)

    def sendStop(self):
        car_cmd_msg = Twist2DStamped()
        car_cmd_msg.header.stamp = rospy.Time.now()
        car_cmd_msg.v = 0
        car_cmd_msg.omega = 0

        self.pub_car_cmd.publish(car_cmd_msg)

    def _cb_velocity(self,vel_msg):
        self._v = vel_msg.v
        self._omega = vel_msg.omega

    def _cb_pose(self,pose_msg):
        self._x = pose_msg.x
        self._y = pose_msg.y
        self._theta = pose_msg.theta

    # Other functions
    def _close(self):
        self.sendStop()
        self.closeCamera()

def _cb_shutdown():
    duck._close()
    # This must be here to prevent segfault
    RR.RobotRaconteurNode.s.Shutdown()

if __name__ == '__main__':
    print "DUCKIEBOT RR BRIDGE"
    print "==================="
    # Parse command line arguments
    parser = argparse.ArgumentParser(
        description='Initialize the Duckiebot')
    parser.add_argument('--port',type=int,default=0,
        help='TCP port to host service on' +\
        '(will auto-generate if not specified)')
    parser.add_argument('--veh', required=True,
        help='The name of the duckiebot being launched')
    parser.add_argument('args', nargs=argparse.REMAINDER)

    args = parser.parse_args(sys.argv[1:])

    veh = args.veh

    # Enable numpy
    RR.RobotRaconteurNode.s.UseNumPy = True

    # Set the node name
    RR.RobotRaconteurNode.s.NodeName = "DuckiebotServer.%s"%veh

    # Register the service def 
    RR.RobotRaconteurNode.s.RegisterServiceType(duckie_servicedef)
    
    # Initialize the object
    duck = DuckiebotHost()

    # Create transport, register it, and start the server
    rospy.loginfo("Registering Transport...")
    t = RR.TcpTransport()
    t.EnableNodeAnnounce(RR.IPNodeDiscoveryFlags_NODE_LOCAL | 
        RR.IPNodeDiscoveryFlags_LINK_LOCAL | 
        RR.IPNodeDiscoveryFlags_SITE_LOCAL)

    RR.RobotRaconteurNode.s.RegisterTransport(t)

    port = args.port
    t.StartServer(port)
    if (port == 0):
        port = t.GetListenPort()

    # Register the service
    rospy.loginfo("Starting Service...")
    RR.RobotRaconteurNode.s.RegisterService("Duckiebot","Duckiebot_Interface.Duckiebot", duck)

    rospy.loginfo("Service Started, connect via \n" +\
        "tcp://localhost:%s/DuckiebotServer.%s/Duckiebot"%(port,veh))

    rospy.on_shutdown(_cb_shutdown)
    rospy.spin()