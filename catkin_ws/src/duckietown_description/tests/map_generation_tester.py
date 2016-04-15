#!/usr/bin/env python
import rospy
import unittest
import rostest
import tf
import os, os.path
import rospkg
import subprocess

class MapGenerationTester(unittest.TestCase):
    def setup(self):
        # Setup the node
        rospy.init_node('duckietown_description_tester_node', anonymous=False)

        # Setup the tf listener
        self.tfl = tf.TransformListener()
        rp = rospkg.RosPack()
        self.path = "{pkg_root}/urdf/test_map_tmp.urdf.xacro".format(pkg_root=rp.get_path("duckietown_description"))

        # Wait for the map generator  to finish saving the file
        timeout = rospy.Time.now() + rospy.Duration(5)  # Wait at most 5 seconds for the node to come up
        while not os.path.isfile(self.path) and not rospy.is_shutdown() and rospy.Time.now() < timeout:
            rospy.sleep(0.1)
        self.assertTrue(os.path.isfile(self.path), "Test timed out while waiting for map file to be generated")

    def test_map_file_generates(self):
        self.setup()  # Setup the node


    def test_csv_to_transform(self):
        self.setup()

        # Launch the duckietown_description
        os.system("roslaunch duckietown_description duckietown_description_node.launch veh:=testbot gui:=false map_file_name:=test_map_tmp.urdf.xacro &")

        # Wait up to 5 seconds for the transform to become available
        self.tfl.waitForTransform("world", "tile_1_1", rospy.Time(), rospy.Duration(5))
        transform_exists = self.tfl.canTransform("world", "tile_1_1", rospy.Time())
        self.assertTrue(transform_exists)

        # Get the param for tile_width
        tile_width = rospy.get_param("~tile_width")
        trans = self.tfl.lookupTransform("world", "tile_1_1", rospy.Time())
        position = trans[0]
        self.assertAlmostEqual(position[0], tile_width)
        self.assertAlmostEqual(position[1], tile_width)
        self.assertAlmostEqual(position[2], 0)

if __name__ == '__main__':
    rostest.rosrun('duckietown_description', 'map_generation_tester', MapGenerationTester)
