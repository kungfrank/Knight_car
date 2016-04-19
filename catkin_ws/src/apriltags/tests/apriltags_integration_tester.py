#!/usr/bin/env python
import rospy
import unittest
import rostest
from duckietown_msgs.msg import AprilTags
from sensor_msgs.msg import CameraInfo, Image
import cv2
from cv_bridge import CvBridge
import tf.transformations as tr
import numpy as np

class ApriltagsIntegrationTester(unittest.TestCase):
    def __init__(self, *args):
        super(ApriltagsIntegrationTester, self).__init__(*args)
        self.msg_tags = AprilTags()
        self.msg_received = False
        self.maxDiff=None

    def setup(self):
        # Setup the node
        rospy.init_node('apriltags_integration_tester', anonymous=True)
        self.msg_received = False
        self.annotations = rospy.get_param("~files")

        # Setup the publisher and subscriber
        self.pub_raw  = rospy.Publisher("~image_raw", Image, queue_size=1, latch=True)
        self.pub_info  = rospy.Publisher("~camera_info", CameraInfo, queue_size=1, latch=True)
        self.sub_tag = rospy.Subscriber( "~apriltags", AprilTags, self.tagCallback)


        # Wait for the node  to finish starting up
        timeout = rospy.Time.now() + rospy.Duration(5) # Wait at most 5 seconds for the node to come up
        while (self.sub_tag.get_num_connections() < 1 or self.pub_info.get_num_connections() < 1
               or self.pub_raw.get_num_connections() < 1) and not rospy.is_shutdown() and rospy.Time.now() < timeout:
            rospy.sleep(0.1)
        self.assertLessEqual(rospy.Time.now(), timeout, "Test timed out while waiting for apriltags nodes to come up")

    def notest_publisher_and_subscriber(self):
        self.setup()    # Setup the node
        self.assertGreaterEqual(self.pub_raw.get_num_connections(), 1, "No connections found on image_raw topic")
        self.assertGreaterEqual(self.pub_info.get_num_connections(), 1, "No connections found on camera_info topic")
        self.assertGreaterEqual(self.sub_tag.get_num_connections(), 1, "No connections found on apriltags topic")

    def tagCallback(self, msg_tag):
        self.msg_tags = msg_tag
        self.msg_received = True

    def send_test_messages(self, filename, id):
        self.msg_received = False
        # Publish the camera info
        msg_info = CameraInfo()
        msg_info.height = 480
        msg_info.width = 640
        msg_info.K = [315.128501, 0.0, 323.069638, 0.0, 320.096636, 218.012581, 0.0, 0.0, 1.0]
        self.pub_info.publish(msg_info)
        # Publish the test image
        img = cv2.imread(filename)
        cvb = CvBridge()
        msg_raw = cvb.cv2_to_imgmsg(img)
        self.pub_raw.publish(msg_raw)

        # Wait for the message to be received
        timeout = rospy.Time.now() + rospy.Duration(5) # Wait at most 5 seconds for the node to reply
        while not self.msg_received and not rospy.is_shutdown() and rospy.Time.now() < timeout:
            rospy.sleep(0.1)
        self.assertLess(rospy.Time.now(), timeout, "Waiting for apriltag detection timed out.")

    def test_apriltag_detection(self):
        self.setup()
        filepath = rospy.get_param("~test_path")

        tag_errs = []
        for file in self.annotations:
            expected_id = self.annotations[file]['id']
            self.send_test_messages(filepath+"/"+file, expected_id)

            # Check for the apriltag with id n
            found = False
            for tag in self.msg_tags.detections:
                if tag.id == expected_id:
                    found = True
                    msg = "Error in file: {file}    ".format(file=file)
                    try:self.assertAlmostEqual(tag.transform.translation.x, self.annotations[file]['x'], delta=0.15)
                    except AssertionError,e: tag_errs.append(msg + str(e))
                    try:self.assertAlmostEqual(tag.transform.translation.y, self.annotations[file]['y'], delta=0.15)
                    except AssertionError,e: tag_errs.append(msg + str(e))
                    try:self.assertAlmostEqual(tag.transform.translation.z, self.annotations[file]['z'], delta=0.15)
                    except AssertionError,e: tag_errs.append(msg + str(e))
                    # Convert the quat to an angle about z
                    rot = tag.transform.rotation
                    ang = tr.euler_from_quaternion((rot.x, rot.y, rot.z, rot.w))[2] #z axis angle only
                    # Allow up to 15 degrees of error margin
                    try:self.assertAlmostEqual(ang, self.annotations[file]['theta'], delta=15*np.pi/180)
                    except AssertionError,e: tag_errs.append(msg + str(e))
            self.assertTrue(found, "Expected apriltag with id={id} not found in file: {file}.".format(id=expected_id,file=file))

        self.assertEqual([], tag_errs)

if __name__ == '__main__':
    rostest.rosrun('apriltags', 'apriltags_integration_tester', ApriltagsIntegrationTester)
