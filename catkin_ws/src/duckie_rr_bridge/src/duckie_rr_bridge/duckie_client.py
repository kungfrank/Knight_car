#!/usr/bin/env python
import RobotRaconteur as RR
RRN=RR.RobotRaconteurNode.s
import numpy

if __name__ == '__main__':
	RRN.UseNumPy=True

	duck = RRN.ConnectService('tcp://<computer>:<port>/DuckiebotServer.<duckiebot>/Duckiebot')

	vel = duck.v
	omega = duck.omega
	x = duck.x
	y = duck.y
	thetha = duck.theta

	lane_pose = duck.lane_pose
	d = lane_pose.d
	phi = lane_pose.phi

	april_tags = duck.april_tags
	if len(april_tags) > 0:
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
	# some processing required to get the image back to normal shape
	# openCV or numpy can probably do this pretty easy...

	duck.closeCamera()






