#!/usr/bin/env python
import RobotRaconteur as RR
RRN=RR.RobotRaconteurNode.s
import numpy
import string
import time

if __name__ == '__main__':
    RRN.UseNumPy=True
    # Register a Local transport
    t1 = RR.LocalTransport()
    RRN.RegisterTransport(t1)

    # Register a TCP transpor
    t2 = RR.TcpTransport()
    RRN.RegisterTransport(t2)
    print "Connecting..."
    duck = RRN.ConnectService("tcp://duckiebot1.local:1234/DuckiebotServer.duckiebot1/Duckiebot")
    print "Connection Successful!"
    
    vel = duck.v
    omega = duck.omega
    x = duck.x
    y = duck.y
    theta = duck.theta
    print "v: {0:0.2f}, omega: {0:0.2f}, x: {0:0.2f}, y: {0:0.2f}, theta: {0:0.2f}".format(vel,omega,x, y,theta)

    print"Lane Pose Retrieved"
    lane_pose = duck.lane_pose
    d = lane_pose.d
    phi = lane_pose.phi

    
    april_tags = duck.april_tags
    if len(april_tags) > 0:
        print "April Tag Retrieved"
        tag = april_tags[0]
        pos = tag.pos
        posx = pos[0]
        posy = pos[1]
        posz = pos[2]

        quat = tag.quat
        quatx = quat[0]
        quaty = quat[1]
        quatz = quat[2]
        quatw = quat[3]

    duck.openCamera()
    img = duck.getImage()
    w = img.width
    h = img.height
    data = img.data
    print "Image acquired"
    # some processing required to get the image back to normal shape
    # openCV or numpy can probably do this pretty easy...

    duck.closeCamera()

    # move forward
    duck.sendCmd(1.0,0)
    time.sleep(1)
    duck.sendStop()
    time.sleep(1)

    # move backwards
    duck.sendCmd(-1.0,0)
    time.sleep(1)
    duck.sendStop()
    time.sleep(1)

    # turn right
    duck.sendCmd(0,1.0)
    time.sleep(1)
    duck.sendStop()
    time.sleep(1)

    # turn left
    duck.sendCmd(0,-1.0)
    time.sleep(1)
    duck.sendStop()






