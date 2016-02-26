#!/usr/bin/env python
import rospy
from sensor_msgs.msg import CompressedImage, Image
from virtual_mirror_npd22 import util
from std_msgs.msg import String
# Initialize the node with rospy
rospy.init_node('virtual_mirror_npd22')

class VirtualMirrorNpd22(object):
    def __init__(self):
        self.node_name = "virtual_mirror_npd22"

	# Publishers
        self.pub_image = rospy.Publisher("~image_mirrored", MirroredImage, queue_size=1)

        # Subscribers
        self.sub_image = rospy.Subscriber("~image", CompressedImage, self.cbImage, queue_size=1)
        # rospy.loginfo("[%s] Initialized." %(self.node_name))

    def processImage(self,image_msg):
	# Decode from compressed image
        # with OpenCV
        image_cv = cv2.imdecode(np.fromstring(image_msg.data, np.uint8), cv2.CV_LOAD_IMAGE_COLOR)

	# Flip image
	# with OpenCV
	image_mirrored = cv2.flip(image_msg.data, dst=None, flipMode=1)

	# Publish mirrored image
        self.pub_image.publish(image_mirrored)

# Read parameter
pub_period = rospy.get_param("~pub_period",1.0)
# Create timer
rospy.Timer(rospy.Duration.from_sec(pub_period),callback)
# spin to keep the script for exiting
rospy.spin()
