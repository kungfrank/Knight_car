#!/usr/bin/env python
import rospkg
import rospy
import yaml
from duckietown_msgs.msg import AprilTags, TagDetection, TagInfo, Vector2D
import numpy as np
import kinematic as k
import tf.transformations as tr
from geometry_msgs.msg import PoseStamped

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
            "t-light-ahead": self.info.T_LIGHT_AHEAD,
            "duck-crossing": self.info.DUCK_CROSSING}

        self.sub_prePros        = rospy.Subscriber("~apriltags_in", AprilTags, self.callback, queue_size=1)
        self.pub_postPros       = rospy.Publisher("~apriltags_out", AprilTags, queue_size=1)
        self.pub_visualize = rospy.Publisher("~tag_pose", PoseStamped, queue_size=1)

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
            camera_x     = 0.05  # x distance from wheel center
            camera_y     = 0.0   #
            camera_z     = 0.1   # height of camera from ground
            camera_theta = 15    # degree of rotation arround y axis
            """
            camera_x     = rospy.get_param("~camera_x")
            camera_y     = rospy.get_param("~camera_y")
            camera_z     = rospy.get_param("~camera_z")
            camera_theta = rospy.get_param("~camera_theta")
            scale_x     = rospy.get_param("~scale_x")
            scale_y     = rospy.get_param("~scale_y")
            scale_z     = rospy.get_param("~scale_z")

            # Define the transforms
            veh_t_camxout = tr.translation_matrix((camera_x, camera_y, camera_z))
            veh_R_camxout = tr.euler_matrix(0, camera_theta*np.pi/180, 0, 'rxyz')
            veh_T_camxout = tr.concatenate_matrices(veh_t_camxout, veh_R_camxout)   # 4x4 Homogeneous Transform Matrix

            camxout_T_camzout = tr.euler_matrix(-np.pi/2,0,-np.pi/2,'rzyx')
            veh_T_camzout = tr.concatenate_matrices(veh_T_camxout, camxout_T_camzout)

            tagzout_T_tagxout = tr.euler_matrix(-np.pi/2, 0, np.pi/2, 'rxyz')

            #Load translation
            trans = detection.transform.translation
            rot = detection.transform.rotation

            camzout_t_tagzout = tr.translation_matrix((trans.x*scale_x, trans.y*scale_y, trans.z*scale_z))
            camzout_R_tagzout = tr.quaternion_matrix((rot.x, rot.y, rot.z, rot.w))
            camzout_T_tagzout = tr.concatenate_matrices(camzout_t_tagzout, camzout_R_tagzout)

            veh_T_tagxout = tr.concatenate_matrices(veh_T_camzout, camzout_T_tagzout, tagzout_T_tagxout)

            # Overwrite transformed value
            (trans.x, trans.y, trans.z) = tr.translation_from_matrix(veh_T_tagxout)
            (rot.x, rot.y, rot.z, rot.w) = tr.quaternion_from_matrix(veh_T_tagxout)

            # TODO bandaid for bug in on-axis rotation for apriltags
            magic_snapper = 35  #Snap angles less than 35deg to 0
            (rx,ry,rz) = tr.euler_from_quaternion((rot.x, rot.y, rot.z, rot.w))
            rz = 0  if abs(rz) < (magic_snapper/180.0 * np.pi) else rz
            (rot.x, rot.y, rot.z, rot.w) = tr.quaternion_from_euler(rx,ry,rz)
            detection.transform.translation = trans
            detection.transform.rotation = rot

            tag_infos.append(new_info)

            # # Debug Print
            # (theta_x, theta_y, theta_z) = (np.array(tr.euler_from_quaternion((rot.x, rot.y, rot.z, rot.w),'rzyx'))*180/np.pi).tolist()
            # rospy.loginfo("[{0}] Translation: [x,y,z]: [{1:.3f}, {2:.3f}, {3:.3f}] Rotation [rx,ry,rz]: [{4:.3f}, {5:.3f}, {6:.3f}]".format("apriltags", trans.x,trans.y,trans.z,theta_x, theta_y, theta_z))
            # tag_infos.append(new_info)
            #
            # (angle, direction, point) = tr.rotation_from_matrix(veh_T_tagxout)
            # rospy.loginfo("[{0}] Axis: [x,y,z]: [{1:.3f}, {2:.3f}, {3:.3f}] Angle: {4:.3f}".format("apriltags", direction[0], direction[1], direction[2], angle*180/np.pi))
            ##### Debug Print TODO REMOVE DEBUG
            (theta_x, theta_y, theta_z) = (np.array(tr.euler_from_quaternion((rot.x, rot.y, rot.z, rot.w),'sxyz'))*180/np.pi).tolist()
            rospy.loginfo("[{0}] Translation: [x,y,z]: [{1:.3f}, {2:.3f}, {3:.3f}] Rotation ('sxyz')[rx,ry,rz]: [{4:.3f}, {5:.3f}, {6:.3f}]".format("apriltags", trans.x,trans.y,trans.z,theta_x, theta_y, theta_z))

            (angle, direction, point) = tr.rotation_from_matrix(tr.quaternion_matrix((rot.x, rot.y, rot.z, rot.w)))
            rospy.loginfo("[{0}] Axis: [x,y,z]: [{1:.3f}, {2:.3f}, {3:.3f}] Angle: {4:.3f}".format("apriltags", direction[0], direction[1], direction[2], angle*180/np.pi))
            pose = PoseStamped()
            pose.header.stamp = rospy.Time.now()
            pose.header.frame_id = "world"
            pose.pose.position = trans
            pose.pose.orientation = rot
            self.pub_visualize.publish(pose)
            #############################################################################

        new_tag_data = AprilTags()
        new_tag_data.detections = msg.detections
        new_tag_data.infos = tag_infos


        # Publish Message
        self.pub_postPros.publish(new_tag_data)

if __name__ == '__main__': 
    rospy.init_node('AprilPostPros',anonymous=False)
    node = AprilPostPros()
    rospy.spin()
