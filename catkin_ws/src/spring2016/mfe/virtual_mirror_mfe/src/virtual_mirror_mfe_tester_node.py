#!/usr/bin/env python
import rospy
import numpy as np
from duckietown_msgs.msg import SegmentList, Segment

class VirtualMirrorTesterNode(object):
    def __init__(self):
        self.pub_compressed_image = rospy.Publisher("~compressed_image", CompressedImage, queue_size=1)
	# Get raw image from decoder node

	# Set flip direction to horizontal
	
	# Flip the image according to flip direction using virtual_mirror_mfe_node

	# Compress the flipped image from virtual_mirror_mfe_node

	# Confirm that it's the same as the test file

	# change flip direction to vertical

	# repeat above steps



    def onShutdown(self):
        rospy.loginfo("[VirtualMirrorMFETesterNode] Shutdown.")

if __name__ == '__main__':
    rospy.init_node('virtual_mirror_mfe_tester',anonymous=False)
    virtual_mirror_mfe_tester_node = VirtualMirrorTesterNode()
    rospy.on_shutdown(virtual_mirror_mfe_tester_node.onShutdown)
