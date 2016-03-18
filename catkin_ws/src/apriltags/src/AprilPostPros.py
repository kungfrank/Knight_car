#!/usr/bin/env python
import rospkg
import rospy
import yaml
from duckietown_msgs.msg import AprilTags, TagDetection, TagInfo

class AprilPostPros(object):
    """ """
    def __init__(self):    
        """ """
        self.node_name = rospy.get_name()
 
        self.sub_prePros        = rospy.Subscriber("apriltags/image_raw", AprilTags, queue_size=1)
        self.pub_postPros       = rospy.Publisher("apriltags", AprilTags, self.callback, queue_size=1)
        
        rospack = rospkg.RosPack()
        self.pkg_path = rospack.get_path('apriltags')
        tags_filepath = self.setupParameter("~tags_file", self.pkg_path+"/tagID/tags_map_001.yaml") 
        
        tags_file = open(tags_filepath, 'r')
        self.tags_dict = yaml.load(tags_file)
        tags_file.close()
        self.info = TagInfo()
        
        self.sign_types = {"StreetName": self.info.S_NAME,
            "TrafficSign": self.info.SIGN,
            "Light": self.info.LIGHT,
            "Localization": self.info.LOCALIZE,
            "Vehicle": self.info.VEHICLE}
        self.traffic_sign_types = {"stop": self.info.STOP, 
            "yield": self.info.YIELD, 
            "no-right-turn": self.info.NO_RIGHT_TURN, 
            "no-left-turn": self.info.NO_LEFT_TURN, 
            "oneway-right": self.info.ONEWAY_RIGHT, 
            "oneway-left": self.info.ONEWAY_LEFT, 
            "4-way-intersect": self.info.FOUR_WAY, 
            "right-T-intersect": self.info.RIGHT_T_INTERSECT, 
            "left-T-intersect": self.info.LEFT_T_INTERSECT,
            "T-intersection": self.info.T_INTERSECTION,
            "do-not-enter": self.info.DO_NOT_ENTER,
            "pedestrian": self.info.PEDESTRIAN,
            "duck-crossing": self.info.DUCK_CROSSING}

    def callback(self, msg):
        """ """ 
        # Load tag detections message
        tag_data = msg.data
        tag_infos = []

        for detection in tag_data.detections:
            new_info = TagInfo()
            new_info.id = int(id_info['id'])
            id_info = self.tags_dict[new_info.id]
            
            # Check yaml file to fill in ID-specific information
            new_info.type = self.sign_types[id_info['tag_type']]
            if new_info.type == self.info.S_NAME:
                new_info.street_name = id_info['street_name']
            elif new_info.type == self.info.SIGN:
                new_info.traffic_sign_type = self.traffic_sign_types[id_info['traffic_sign_type']]
            elif new_info.type == self.info.VEHICLE:
                new_info.vehicle_name = id_info['vehicle_name']
             
            # TODO: Add relative pose estimation
            tag_infos.append(new_info)
        
        new_tag_data = AprilTags()
        new_tag_data.detections = tag_data.detections
        new_tag_data.infos = tag_infos

        # Publish Message
        self.pub_postPros.publish(new_tag_data)

if __name__ == '__main__': 
    rospy.init_node('AprilPostPros',anonymous=False)
    node = AprilPostPros()
    rospy.spin()
