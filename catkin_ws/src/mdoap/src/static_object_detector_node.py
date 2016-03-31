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
    CONE = [np.array(x, np.uint8) for x in [[0,80,80], [22, 255,255]] ]
    DUCK = [np.array(x, np.uint8) for x in [[25,100,150], [35, 255, 255]] ]
    terms = {0:"cone", 1:"duck"}

    def get_filtered_contours(self,img, contour_type):
        hsv_img = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        
        if contour_type == "CONE":
            frame_threshed = cv2.inRange(hsv_img, *self.CONE)
            ret,thresh = cv2.threshold(frame_threshed,22,255,0)
        elif contour_type == "DUCK_COLOR":
            frame_threshed = cv2.inRange(hsv_img, *self.DUCK)
            ret,thresh = cv2.threshold(frame_threshed,30,255,0)
        elif contour_type == "DUCK_CANNY":
            frame_threshed = cv2.inRange(hsv_img, *self.DUCK)
            frame_threshed = cv2.adaptiveThreshold(frame_threshed,255,\
                    cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY,5,2)
            thresh = cv2.Canny(frame_threshed, 100,200)
        else:
            return 
        
        filtered_contours = []
        
        contours, hierarchy = cv2.findContours(\
                thresh,cv2.RETR_CCOMP,cv2.CHAIN_APPROX_SIMPLE)
        contour_area = [ (cv2.contourArea(c), (c) ) for c in contours]
        contour_area = sorted(contour_area,reverse=True, key=lambda x: x[0])

        height,width = img.shape[:2]
        for (area,(cnt)) in contour_area:
        # plot box around contour
            x,y,w,h = cv2.boundingRect(cnt)
            box = (x,y,w,h)
            d =  0.5*(x-width/2)**2 + (y-height)**2 
            if not(h>15 and w >15 and d  < 120000):
                    continue
            if contour_type =="DUCK_COLOR": # extra filtering to remove lines
                val = cv2.arcLength(cnt,True)**2/ cv2.contourArea(cnt)
                if val > 35: continue
                rect = cv2.minAreaRect(cnt)
                ctr, sides, deg = rect
                val  = 0.5*cv2.arcLength(cnt,True) / (w**2+h**2)**0.5
                if val < 1.12: continue
                #if area > 1000: continue

            mask = np.zeros(thresh.shape,np.uint8)
            cv2.drawContours(mask,[cnt],0,255,-1)
            mean_val = cv2.mean(img,mask = mask)
            aspect_ratio = float(w)/h
            filtered_contours.append( (cnt, box, d, aspect_ratio, mean_val) )
        return filtered_contours


    def contour_match(self, img):
        '''
        Returns 1. Image with bounding boxes added
                2. an ObstacleImageDetectionList
        '''

        object_list = ObstacleImageDetectionList()
        object_list.list = []

        height,width = img.shape[:2]
        object_list.imwidth = width
        object_list.imheight = height
        
        # get filtered contours
        cone_contours = self.get_filtered_contours(img, "CONE")
        duck_contours = self.get_filtered_contours(img, "DUCK_COLOR")

        all_contours = [cone_contours, duck_contours]
        for i, contours in enumerate(all_contours):
            for (cnt, box, ds, aspect_ratio, mean_color)  in contours:
                            
                # plot box around contour
                x,y,w,h = box
                font = cv2.FONT_HERSHEY_SIMPLEX
                cv2.putText(img,self.terms[i], (x,y), font, 0.5,mean_color,4)
                cv2.rectangle(img,(x,y),(x+w,y+h), mean_color,2)
                
                r = Rect()
                r.x = x
                r.y = y
                r.w = w
                r.h = h

                t = ObstacleType()
                t.type = i

                d = ObstacleImageDetection()
                d.bounding_box = r
                d.type = t

                object_list.list.append(d);
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
