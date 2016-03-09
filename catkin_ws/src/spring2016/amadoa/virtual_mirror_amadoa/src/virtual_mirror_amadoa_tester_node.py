#!/usr/bin/env python
import rospy
import numpy as np
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import CompressedImage
from numpy import array

class MirrorTesterNode(object):
    def __init__(self):
        self.node_name = "virtual_mirror_amadoa_tester_node"
        # self.flip_direction = self.setupParam("~flip_direction", "horz") # horz or vert
        # Set up publisher to publish test image
        self.pub_mirror = rospy.Publisher("~test_image",CompressedImage,queue_size=1, latch=True)

        # Set up subscriber to test returned image
        self.subscriber = rospy.Subscriber("~mirrored_image", CompressedImage, self.imageCallback)


    def setupParam(self, param_name,default_value):
        value = rospy.get_param(param_name,default_value)
        rospy.set_param(param_name,value) #Write to parameter server for transparancy
        rospy.loginfo("[%s] %s = %s " %(self.node_name, param_name,value))
        return value

    def publishTestImage(self):
        original_image_file = self.setupParam("~original_image_file","")
        original_img = cv2.imread(original_image_file, cv2.CV_LOAD_IMAGE_COLOR)

        #### Create CompressedIamge ####
        out = CompressedImage()
        out.header.stamp = rospy.Time.now()
        out.format = "png"
        np_arr_out = cv2.imencode('.png', original_img)[1]
        out.data = np.array(np_arr_out).tostring()

        # Publish new image
        self.pub_mirror.publish(out)
        print "published image"

    def imageCallback(self, msg):
        # Load reference image
        flipped_image_file = self.setupParam("~flipped_image_file","")
        flipped_img = cv2.imread(flipped_image_file, cv2.CV_LOAD_IMAGE_COLOR)

        # Decode returned image
        np_arr = np.fromstring(msg.data, np.uint8)
        returned_image = cv2.imdecode(np_arr, cv2.CV_LOAD_IMAGE_COLOR)

        # print (returned_image - flipped_img)

        # print "in: ",flipped_img.shape, "out: ", returned_image.shape
        cv2.imwrite("/home/amado/pics_duckietown/Test_images/01_diff.jpg",cv2.subtract(flipped_img, returned_image))


        if np.array_equal(returned_image, flipped_img):
            rospy.loginfo("Test passed successfully!")
        else:
            rospy.logwarn("Test failed :(")

    def onShutdown(self):
        rospy.loginfo("[virtual_mirror_amadoa_node] Shutdown.")

if __name__ == '__main__':
    rospy.init_node("virtual_mirror_amadoa_tester_node",anonymous=False)
    mirror_tester_node = MirrorTesterNode()
    mirror_tester_node.publishTestImage()
    rospy.on_shutdown(mirror_tester_node.onShutdown)
    rospy.spin()
