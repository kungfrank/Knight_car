#!/usr/env/bin python
#import cv2
import rospy
import roslib; roslib.load_manifest("virtual_mirror_onasafari")
from sensor_msgs.msg import CompressedImage, Image
import threading

class VirtualMirror:
    def __init__(self):
        self.thread_lock = threading.Lock()
        self.camera_sub = rospy.Subscriber("~image_raw", CompressedImage, self.cbImage)
        self.mirror_pub = rospy.Publisher("~mirror", Image, queue_size=1)

    def cbImage(self, image_msg):
        # Start a daemon thread to process the image
        thread = threading.Thread(target=self.processImage,args=(image_msg,))
        thread.setDaemon(True)
        thread.start()
        self.camera_sub.unregister()
        self.processImage(image_msg)

        # Returns rightaway

    def processImage(self, image_msg):
        data = image_msg.data
        import pdb ; pdb.set_trace()
        

if __name__=="__main__":
    rospy.init_node("virtual_mirror")
    VM = VirtualMirror()
    rospy.spin()
