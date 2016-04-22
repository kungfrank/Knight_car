#!/usr/bin/env python
import rospkg
import rospy
import yaml
from apriltags_ros.msg import AprilTagDetectionArray
import numpy as np
import tf.transformations as tr
from geometry_msgs.msg import PoseStamped

class AprilPostPros(object):
    """ """
    def __init__(self):    
        """ """
        self.node_name = "apriltags_postprocessing_node"

        # Load parameters
        self.camera_x     = self.setupParam("~camera_x", 065)
        self.camera_y     = self.setupParam("~camera_y", 0.0)
        self.camera_z     = self.setupParam("~camera_z", 0.11)
        self.camera_theta = self.setupParam("~camera_theta", 19.0)
        self.scale_x     = self.setupParam("~scale_x", 1)
        self.scale_y     = self.setupParam("~scale_y", 1)
        self.scale_z     = self.setupParam("~scale_z", 1)
        self.bandaid     = self.setupParam("~bandaid", False)
        self.snap_angle = self.setupParam("~snap_angle", 35.0)

        self.sub_prePros        = rospy.Subscriber("~apriltags_in", AprilTagDetectionArray, self.callback, queue_size=1)
        self.pub_postPros       = rospy.Publisher("~apriltags_out", AprilTagDetectionArray, queue_size=1)
        self.pub_visualize = rospy.Publisher("~tag_pose", PoseStamped, queue_size=1)

        rospy.loginfo("[%s] has started", self.node_name)

    def setupParam(self,param_name,default_value):
        value = rospy.get_param(param_name,default_value)
        rospy.set_param(param_name,value) #Write to parameter server for transparancy
        rospy.loginfo("[%s] %s = %s " %(self.node_name,param_name,value))
        return value

    def callback(self, msg):
        # Load tag detections message
        for detection in msg.detections:

            # Define the transforms
            veh_t_camxout = tr.translation_matrix((self.camera_x, self.camera_y, self.camera_z))
            veh_R_camxout = tr.euler_matrix(0, self.camera_theta*np.pi/180, 0, 'rxyz')
            veh_T_camxout = tr.concatenate_matrices(veh_t_camxout, veh_R_camxout)   # 4x4 Homogeneous Transform Matrix

            camxout_T_camzout = tr.euler_matrix(-np.pi/2,0,-np.pi/2,'rzyx')
            veh_T_camzout = tr.concatenate_matrices(veh_T_camxout, camxout_T_camzout)

            tagzout_T_tagxout = tr.euler_matrix(-np.pi/2, 0, np.pi/2, 'rxyz')

            #Load translation
            trans = detection.pose.pose.position
            rot = detection.pose.pose.orientation

            camzout_t_tagzout = tr.translation_matrix((trans.x*self.scale_x, trans.y*self.scale_y, trans.z*self.scale_z))
            camzout_R_tagzout = tr.quaternion_matrix((rot.x, rot.y, rot.z, rot.w))
            camzout_T_tagzout = tr.concatenate_matrices(camzout_t_tagzout, camzout_R_tagzout)

            veh_T_tagxout = tr.concatenate_matrices(veh_T_camzout, camzout_T_tagzout, tagzout_T_tagxout)

            # Overwrite transformed value
            (trans.x, trans.y, trans.z) = tr.translation_from_matrix(veh_T_tagxout)
            (rot.x, rot.y, rot.z, rot.w) = tr.quaternion_from_matrix(veh_T_tagxout)

            # TODO bandaid for bug in on-axis rotation for apriltags
            if self.bandaid:
                magic_snapper = self.snap_angle  #Snap angles less than Xdeg to 0
                (rx,ry,rz) = tr.euler_from_quaternion((rot.x, rot.y, rot.z, rot.w))
                rz = 0  if abs(rz) < (magic_snapper/180.0 * np.pi) else rz
            (rot.x, rot.y, rot.z, rot.w) = tr.quaternion_from_euler(rx,ry,rz)
            detection.pose.pose.position = trans
            detection.pose.pose.orientation = rot

            # # Debug Print
            # (theta_x, theta_y, theta_z) = (np.array(tr.euler_from_quaternion((rot.x, rot.y, rot.z, rot.w),'rzyx'))*180/np.pi).tolist()
            # rospy.loginfo("[{0}] Translation: [x,y,z]: [{1:.3f}, {2:.3f}, {3:.3f}] Rotation [rx,ry,rz]: [{4:.3f}, {5:.3f}, {6:.3f}]".format("apriltags", trans.x,trans.y,trans.z,theta_x, theta_y, theta_z))
            # tag_infos.append(new_info)
            #
            # (angle, direction, point) = tr.rotation_from_matrix(veh_T_tagxout)
            # rospy.loginfo("[{0}] Axis: [x,y,z]: [{1:.3f}, {2:.3f}, {3:.3f}] Angle: {4:.3f}".format("apriltags", direction[0], direction[1], direction[2], angle*180/np.pi))
            ##### Debug Print TODO REMOVE DEBUG Pringing
            # (theta_x, theta_y, theta_z) = (np.array(tr.euler_from_quaternion((rot.x, rot.y, rot.z, rot.w),'sxyz'))*180/np.pi).tolist()
            # rospy.loginfo("[{0}] Translation: [x,y,z]: [{1:.3f}, {2:.3f}, {3:.3f}] Rotation ('sxyz')[rx,ry,rz]: [{4:.3f}, {5:.3f}, {6:.3f}]".format("apriltags", trans.x,trans.y,trans.z,theta_x, theta_y, theta_z))
            #
            # (angle, direction, point) = tr.rotation_from_matrix(tr.quaternion_matrix((rot.x, rot.y, rot.z, rot.w)))
            # rospy.loginfo("[{0}] Axis: [x,y,z]: [{1:.3f}, {2:.3f}, {3:.3f}] Angle: {4:.3f}".format("apriltags", direction[0], direction[1], direction[2], angle*180/np.pi))
            # pose = PoseStamped()
            # pose.header.stamp = rospy.Time.now()
            # pose.header.frame_id = "world"
            # pose.pose.position = trans
            # pose.pose.orientation = rot
            # self.pub_visualize.publish(pose)
            #############################################################################

        # Publish Message
        self.pub_postPros.publish(msg)

if __name__ == '__main__': 
    rospy.init_node('AprilPostPros',anonymous=False)
    node = AprilPostPros()
    rospy.spin()
