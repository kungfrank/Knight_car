#!/usr/bin/env python
import rospy
import time
from led_detection.LEDDetector import LEDDetector
from duckietown_msgs.msg import Vector2D, LEDDetection, LEDDetectionArray
from sensor_msgs.msg import CompressedImage
from duckietown_utils.bag_logs import numpy_from_ros_compressed
import numpy as np

class LEDDetectorNode(object):
    def __init__(self):
        self.first_timestamp = None
        self.data = []
        self.capture_time = 3 # capture time
        self.capture_finished = False
        
        print('Constructing LEDDetector')
        self.node_name = rospy.get_name()
        self.pub_detections = rospy.Publisher("~raw_led_detection",LEDDetectionArray,queue_size=1)
        # TODO get veh parameter
        self.sub_cam = rospy.Subscriber("/argo/camera_node/image/compressed",CompressedImage, self.camera_callback)

    def camera_callback(self, msg):
        float_time = msg.header.stamp.to_sec()

        if self.first_timestamp is None:
            self.first_timestamp = msg.header.stamp.to_sec()
        # TODO sanity check rel_time positive, restart otherwise 
        rel_time = float_time - self.first_timestamp

        if rel_time < self.capture_time:
            rgb = numpy_from_ros_compressed(msg)
            print('Capturing frame %s' %rel_time)
            self.data.append({'timestamp': float_time, 'rgb': rgb})
        elif not self.capture_finished:
            self.capture_finished = True
            self.process_and_publish()
            # TODO can also remove the subscriber

    def process_and_publish(self):
    	H, W, _ = self.data[0]['rgb'].shape
        n = len(self.data)
        dtype = [
            ('timestamp', 'float'),
            ('rgb', 'uint8', (H, W, 3)),
        ]
        images = np.zeros((n,), dtype=dtype)
        for i, v in enumerate(self.data):
            images[i]['timestamp'] = v['timestamp']
            images[i]['rgb'][:] = v['rgb']
        
        det = LEDDetector()
        rgb0 = self.data[0]['rgb']
        mask = np.ones(dtype='bool', shape=rgb0.shape)
        tic = time.time()
        result = det.detect_led(images, mask, [1.0, 1.5, 2.0], 5)
        self.pub_detections.publish(result)
        toc = time.time()-tic
        print('Done. Time taken: %.2f'%toc)

if __name__ == '__main__':
    rospy.init_node('LED_detector_node',anonymous=False)
    node = LEDDetectorNode()
    rospy.spin()

