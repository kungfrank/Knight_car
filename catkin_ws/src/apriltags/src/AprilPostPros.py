#!/usr/bin/env python
import rospkg
import rospy
import yaml
from duckietown_msgs.msg import AprilTags, TagDetection, TagInfo, Vector2D
import numpy as np
import kinematic as k

class AprilPostPros(object):
    """ """
    def __init__(self):    
        """ """
        self.node_name = rospy.get_name()

        rospack = rospkg.RosPack()
        self.pkg_path = rospack.get_path('apriltags')
        tags_filepath = self.setupParam("~tags_file", self.pkg_path+"/apriltagsDB/apriltagsDB.yaml") # No tags_file input atm., so default value is used
        self.loc = self.setupParam("~loc", -1) # -1 if no location is given
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

        self.sub_prePros        = rospy.Subscriber("~apriltags_in", AprilTags, self.callback, queue_size=1)
        self.pub_postPros       = rospy.Publisher("~apriltags_out", AprilTags, queue_size=1)

    def setupParam(self,param_name,default_value):
        value = rospy.get_param(param_name,default_value)
        rospy.set_param(param_name,value) #Write to parameter server for transparancy
        rospy.loginfo("[%s] %s = %s " %(self.node_name,param_name,value))
        return value

    def callback(self, msg):
        """ """ 
        # Load tag detections message
        tag_infos = []

        for detection in msg.detections:
            new_info = TagInfo()
            #new_location_info = Vector2D()
            new_info.id = int(detection.id)
            id_info = self.tags_dict[new_info.id]
            
            # Check yaml file to fill in ID-specific information
            new_info.tag_type = self.sign_types[id_info['tag_type']]
            if new_info.tag_type == self.info.S_NAME:
                new_info.street_name = id_info['street_name']
            elif new_info.tag_type == self.info.SIGN:
                new_info.traffic_sign_type = self.traffic_sign_types[id_info['traffic_sign_type']]
            elif new_info.tag_type == self.info.VEHICLE:
                new_info.vehicle_name = id_info['vehicle_name']
            
            # TODO: Implement location more than just a float like it is now.
            # location is now 0.0 if no location is set which is probably not that smart
            if self.loc == 226:
                l = (id_info['location_226'])
                if l is not None:
                    new_info.location = l
            elif self.loc == 316:
                l = (id_info['location_316'])
                if l is not None:
                    new_info.location = l
            
            
            #######################
            # Localization stuff
            #######################
            
            
            """
            scale        = 0.31
            camera_x     = 0.05  # x distance from wheel center
            camera_y     = 0.0   #
            camera_z     = 0.1   # height of camera from ground
            camera_theta = 15    # degree of rotation arround y axis
            """
            scale        = rospy.get_param("~scale")
            camera_x     = rospy.get_param("~camera_x")
            camera_y     = rospy.get_param("~camera_y")
            camera_z     = rospy.get_param("~camera_z")
            camera_theta = rospy.get_param("~camera_theta")
            
            
            #Load translation
            x = detection.transform.translation.x
            y = detection.transform.translation.y
            z = detection.transform.translation.z
            
            t_tc_Fc = k.Vector( x , y , z ) # translation tags(t) w/ camera(c) expressed in camera frame (Fc)
            
            # Scale for april tag size
            t_tc_Fc = t_tc_Fc * scale
            
            #Load rotation
            x = detection.transform.rotation.x
            y = detection.transform.rotation.y 
            z = detection.transform.rotation.z
            w = detection.transform.rotation.w
            e = k.Vector( x , y , z )
            Q_Ftag_Fold = k.Quaternion( e , w )
            
            # New tag orientation reference (zero when facing camera) w/ to old tag ref used by the lib
            C_Ft_Ftag = k.RotationMatrix( np.matrix([[0,0,-1],[-1,0,0],[0,1,0]]) )
            Q_Ft_Ftag = C_Ft_Ftag.toQuaternion()
            
            # Rotation of old ref frame used by the lib w/ to camera frame
            C_Fold_Fc = k.RotationMatrix( np.matrix([[0,-1,0],[0,0,-1],[1,0,0]]) )
            Q_Fold_Fc = C_Fold_Fc.toQuaternion()
            
            # Camera localization
            t_cv_Fv = k.Vector( camera_x , camera_y , camera_z ) # translation of camera w/ vehicle origin in vehicle frame
            C_Fc_Fv = k.euler2RotationMatrix(0,camera_theta,0)   # Rotation   of camera frame w/ vehicle frame
            Q_Fc_Fv = C_Fc_Fv.toQuaternion()
            
            # Compute tag orientation in vehicle frame
            Q_Ft_Fv =  Q_Fc_Fv * Q_Fold_Fc * Q_Ftag_Fold * Q_Ft_Ftag
            
            # Compute position of tag in vehicle frame expressed in vehicle frame
            C_Fv_Fc = - C_Fc_Fv # inverse transform
            t_tc_Fv = C_Fv_Fc * t_tc_Fc
            t_tv_Fv = t_tc_Fv + t_cv_Fv
            
            # Overwrite transformed value
            detection.transform.translation.x = t_tv_Fv.x
            detection.transform.translation.y = t_tv_Fv.y
            detection.transform.translation.z = t_tv_Fv.z
            detection.transform.rotation.x    = Q_Ft_Fv.e.x
            detection.transform.rotation.y    = Q_Ft_Fv.e.y
            detection.transform.rotation.z    = Q_Ft_Fv.e.z
            detection.transform.rotation.w    = Q_Ft_Fv.n
            
            # Debug Print
            #A_read       = Q_Ftag_Fold.toAngleAxis()
            #A_Ft_Fv      = Q_Ft_Fv.toAngleAxis()
            #print 'Rotation Read'
            #A_read()
            #print 'Rotation in Vehicle Frame'
            #A_Ft_Fv()

            tag_infos.append(new_info)
        
        new_tag_data = AprilTags()
        new_tag_data.detections = msg.detections
        new_tag_data.infos = tag_infos


        # Publish Message
        self.pub_postPros.publish(new_tag_data)

if __name__ == '__main__': 
    rospy.init_node('AprilPostPros',anonymous=False)
    node = AprilPostPros()
    rospy.spin()
