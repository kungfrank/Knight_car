#!/usr/bin/env python
import rospy
import unittest
import rostest
from duckietown_msgs.msg import AprilTagsWithInfos
from sensor_msgs.msg import CameraInfo, Image
import cv2
from cv_bridge import CvBridge
import tf.transformations as tr
import numpy as np


class ApriltagsIntegrationTester(unittest.TestCase):
    def __init__(self, *args):
        super(ApriltagsIntegrationTester, self).__init__(*args)
        self.msg_tags = AprilTagsWithInfos()
        self.msg_received = False
        self.maxDiff = None

    def setup(self):
        # Setup the node
        rospy.init_node('apriltags_integration_tester', anonymous=False)
        self.msg_received = False
        self.annotations = rospy.get_param("~files")

        # Setup the publisher and subscriber
        self.pub_raw = rospy.Publisher("~image_raw", Image, queue_size=1, latch=True)
        self.pub_info = rospy.Publisher("~camera_info", CameraInfo, queue_size=1, latch=True)
        self.sub_tag = rospy.Subscriber("~apriltags", AprilTagsWithInfos, self.tagCallback)

        # Wait for the node  to finish starting up
        timeout = rospy.Time.now() + rospy.Duration(5)  # Wait at most 5 seconds for the node to come up
        while (self.sub_tag.get_num_connections() < 1 or self.pub_info.get_num_connections() < 1
               or self.pub_raw.get_num_connections() < 1) and not rospy.is_shutdown() and rospy.Time.now() < timeout:
            rospy.sleep(0.1)
        self.assertLessEqual(rospy.Time.now(), timeout, "Test timed out while waiting for apriltags nodes to come up")

    def test_publisher_and_subscriber(self):
        self.setup()  # Setup the node
        self.assertGreaterEqual(self.pub_raw.get_num_connections(), 1, "No connections found on image_raw topic")
        self.assertGreaterEqual(self.pub_info.get_num_connections(), 1, "No connections found on camera_info topic")
        self.assertGreaterEqual(self.sub_tag.get_num_connections(), 1, "No connections found on apriltags topic")

    def tagCallback(self, msg_tag):
        self.msg_tags = msg_tag
        self.msg_received = True

    def send_test_messages(self, filename):
        self.msg_received = False
        # Publish the camera info TODO make this a field in the annotations file to dictate the source calibration file
        msg_info = CameraInfo()
        msg_info.height = 480
        msg_info.width = 640
        msg_info.distortion_model = "plumb_bob"
        msg_info.D = [-0.28048157543793056, 0.05674481026365553, -0.000988764087143394, -0.00026869128565781613, 0.0]
        msg_info.K = [315.128501, 0.0, 323.069638, 0.0, 320.096636, 218.012581, 0.0, 0.0, 1.0]
        msg_info.R = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        msg_info.P = [217.48876953125, 0.0, 321.3154072384932, 0.0, 0.0, 250.71084594726562, 202.30416165274983, 0.0,
                      0.0, 0.0, 1.0, 0.0]
        msg_info.roi.do_rectify = False

        # Publish the test image
        img = cv2.imread(filename)
        cvb = CvBridge()
        msg_raw = cvb.cv2_to_imgmsg(img, encoding="bgr8")
        self.pub_info.publish(msg_info)
        self.pub_raw.publish(msg_raw)

        # Wait for the message to be received
        timeout = rospy.Time.now() + rospy.Duration(10)  # Wait at most 5 seconds for the node to reply
        while not self.msg_received and not rospy.is_shutdown() and rospy.Time.now() < timeout:
            rospy.sleep(0.1)
        self.assertLess(rospy.Time.now(), timeout, "Waiting for apriltag detection timed out.")

    def test_apriltag_detection(self):
        self.setup()
        filepath = rospy.get_param("~test_path")

        tag_errs = []
        id_errs = []
        for file in self.annotations:
            expected = self.annotations[file]
            rospy.loginfo("====Testing file: %s", filepath + "/" + file)
            self.send_test_messages(filepath + "/" + file)

            # Check for the apriltag with id n
            found = False
            for tag in self.msg_tags.detections:
                if tag.id == expected['id']:
                    found = True
                    trans = tag.pose.pose.position
                    rot = tag.pose.pose.orientation
                    ang = tr.euler_from_quaternion((rot.x, rot.y, rot.z, rot.w))[2]  # z axis angle only

                    msg = "Error in file: {file}    ".format(file=file)
                    msg += "Actual [x,y,z,theta]: [{x:.3f}, {y:.3f}, {z:.3f}, {theta:.3f}]    ".format(x=trans.x,
                            y=trans.y, z=trans.z, theta=ang)
                    msg += "Expected: [{x:.3f}, {y:.3f}, {z:.3f}, {theta:.3f}]    ".format(x=expected['x'],
                            y=expected['y'], z=expected['z'], theta=expected['theta'])
                    try:
                        self.assertAlmostEqual(trans.x, expected['x'], delta=0.1)
                    except AssertionError, e:
                        tag_errs.append(msg + "x: " + str(e))
                    try:
                        self.assertAlmostEqual(trans.y, expected['y'], delta=0.1)
                    except AssertionError, e:
                        tag_errs.append(msg + "y: " + str(e))
                    try:
                        self.assertAlmostEqual(trans.z, expected['z'], delta=0.1)
                    except AssertionError, e:
                        tag_errs.append(msg + "z: " + str(e))
                    try:
                        self.assertAlmostEqual(ang, expected['theta'],
                                               delta=15 * np.pi / 180)  # Allow up to 15 degrees err
                    except AssertionError, e:
                        tag_errs.append(msg + "theta:" + str(e))
            try:
                self.assertTrue(found,
                                "Expected apriltag with id={id} not found in file: {file}.".format(id=expected['id'],
                                                                                                   file=file))
            except AssertionError, e:
                id_errs.append(str(e))
        self.assertEqual([], id_errs)
        self.assertEqual([], tag_errs)


if __name__ == '__main__':
    rostest.rosrun('apriltags', 'apriltags_integration_tester', ApriltagsIntegrationTester)
