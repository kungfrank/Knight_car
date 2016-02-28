#!/usr/bin/env python

# numpy and scipy
import numpy as np
#from scipy.ndimage import filters

# OpenCV
import cv2, sys
from cv_bridge import CvBridge, CvBridgeError

#ROS Libraries
#import roslib
import rospy

#ROS Messages
#from std_msgs.msg import String #Imports msg
from sensor_msgs.msg import CompressedImage, Image

class image_averager:

    def __init__(self):
        self.bridge = CvBridge()
        # Create publisher
        # publisher = rospy.Publisher("~rgb_out",CompressedImage,queue_size=1)
        self.publisher = rospy.Publisher("~rgb_out",Image,queue_size=1)
        # Create subscriber
        self.subscriber = rospy.Subscriber("~rgb_in",CompressedImage,self.callback)
        #print 'we made it before the spin'
        #rospy.spin() #Keeps the script for exiting
        self.avgImg = None
        self.numImg = 0

    # Define callback function
    def callback(self,ros_data):
        #print 'received image of type: "%s"' % ros_data.format
        #compressedImage first gets converted into a numpy array
        try:
            np_arr = np.fromstring(ros_data.data, np.uint8)
            #decode the image into a raw cv2 image (numpy.ndarray)
            rgb_in_u8 = cv2.imdecode(np_arr, cv2.CV_LOAD_IMAGE_COLOR)
        except CvBridgeError as e:
            print(e)

        rgb_in_f = rgb_in_u8.astype('float')
        rgb_in_fn = cv2.normalize(rgb_in_f, None, 0.0,255.0, cv2.NORM_MINMAX)
        
        if self.avgImg is not None:
            alpha = float(self.numImg)/float(self.numImg + 1)
            beta = (1.0 / float(self.numImg + 1))
            rgb_out_f = cv2.addWeighted(self.avgImg,alpha,rgb_in_fn,beta,0.0)
            rgb_out = rgb_out_f.astype('uint8')
            self.avgImg = rgb_out_f
            msg = self.bridge.cv2_to_imgmsg(rgb_out,"bgr8")
            try:
                self.publisher.publish(msg)
            except CvBridgeError as e:
                print(e)
            ##### Create CompressedImage ####
            #msg = CompressedImage()
            #msg.header.stamp = rospy.Time.now()
            #msg.format = "jpeg"
            #msg.data = np.array(cv2.imencode('.jpg', rgb_out)[1]).tostring()
            ## Publish new image
            #publisher.publish(msg)	
            #print 'send image of type: "%s"' % msg.format
        else:
            self.avgImg = rgb_in_fn
        
        self.numImg += 1

def main(args):
    
    rospy.init_node('image_averager', anonymous=True)
    ic = image_averager()
    # Initialize the node with rospy
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
