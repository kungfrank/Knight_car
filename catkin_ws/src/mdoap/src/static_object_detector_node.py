#!/usr/bin/env python
import cv2
import numpy as np
import rospy
from sensor_msgs.msg import CompressedImage, Image
from std_msgs.msg import Float32
from cv_bridge import CvBridge, CvBridgeError
from duckietown_msgs.msg import ObstacleImageDetection, ObstacleImageDetectionList, ObstacleType, Rect
import sys
import threading


class Matcher:
    def __init__(self):
        pass

    def contour_match(self, img):
        '''
        Returns 1. Image with bounding boxes added
                2. an ObstacleImageDetectionList
        '''

        object_list = ObstacleImageDetectionList()
        object_list.list = []

        height,width = img.shape[:2]
        hsv_img = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        COLOR_MIN = np.array([0, 80, 80],np.uint8)
        COLOR_MAX = np.array([22, 255, 255],np.uint8)
        frame_threshed = cv2.inRange(hsv_img, COLOR_MIN, COLOR_MAX)
        imgray = frame_threshed
        ret,thresh = cv2.threshold(frame_threshed,22,255,0)
        try:
            contours, hierarchy = cv2.findContours(\
                    thresh,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)

            # Find the index of the largest contour
            areas = [cv2.contourArea(c) for c in contours]
            max_index = np.argmax(areas)
            cnt = contours[max_index]
            contour_area = [ (cv2.contourArea(c), (c) ) for c in contours]
            #contour_area.sort()
            contour_area = sorted(contour_area,reverse=True, key=lambda x: x[0])
            for (area,(cnt)) in contour_area[:10]:
            # plot box around contour
                x,y,w,h = cv2.boundingRect(cnt)
                d =  0.5*(x-width/2)**2 + (y-height)**2 
                if h>15 and w >15 and d  < 120000:
                    r = Rect()
                    r.x = x
                    r.y = y
                    r.w = w
                    r.h = h
                    t = ObstacleType()
                    #TODO(??): Assign type based on color
                    t.type = ObstacleType.CONE
                    d = ObstacleImageDetection()
                    d.bounding_box = r
                    d.type = t

                    object_list.list.append(d);
                    cv2.rectangle(img,(x,y),(x+w,y+h),(0,255,0),2)
        except:
            print "contr err"
        return img, object_list

class StaticObjectDetectorNode:
    def __init__(self, target_img="cone.png"):
        self.name = 'static_object_detector_node'
        

        self.tm = Matcher()
        self.thread_lock = threading.Lock()
        self.sub_image = rospy.Subscriber("~image_compressed", CompressedImage, self.cbImage, queue_size=1)
        self.pub_image = rospy.Publisher("~cone_detection_image", Image, queue_size=1)
        self.pub_detections_list = rospy.Publisher("~detection_list", ObstacleImageDetectionList, queue_size=1)
        self.bridge = CvBridge()

        rospy.loginfo("[%s] Initialized." %(self.name))

    def cbImage(self,image_msg):
        thread = threading.Thread(target=self.processImage,args=(image_msg,))
        thread.setDaemon(True)
        thread.start()

    def processImage(self, image_msg):
        if not self.thread_lock.acquire(False):
            return

        np_arr = np.fromstring(image_msg.data, np.uint8)
        
        image_cv = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        img, detections = self.tm.contour_match(image_cv)
        detections.header.stamp = image_msg.header.stamp
        detections.header.frame_id = image_msg.header.frame_id
        self.pub_detections_list.publish(detections)
        height,width = img.shape[:2]
        try:
            self.pub_image.publish(self.bridge.cv2_to_imgmsg(img, "bgr8"))
        except CvBridgeError as e:
            print(e)

        self.thread_lock.release()

if __name__=="__main__":
	rospy.init_node('static_object_detector_node')
	node = StaticObjectDetectorNode()
	rospy.spin()