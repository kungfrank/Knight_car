#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <duckietown_msgs/TagDetection.h>
#include <duckietown_msgs/AprilTags.h>

#include <opencv/cv.h>
#include <cv_bridge/cv_bridge.h>
#include <tf_conversions/tf_eigen.h>
#include <tf/transform_datatypes.h>

#include "AprilTags/TagDetector.h"
#include "AprilTags/Tag36h11.h"

class AprilTagDetectorNode{
public:
  ros::NodeHandle nh_p_;
  ros::Publisher pub_detection_;

  ros::Subscriber sub_image_;
  ros::Subscriber sub_camera_info_;

  AprilTags::TagDetector tag_detector_;
  sensor_msgs::CameraInfo camera_info_;
  bool has_camera_info_;
  double tag_size_; //Size of the tag in meters

  AprilTagDetectorNode(const ros::NodeHandle& nh): nh_p_(nh), tag_detector_(AprilTags::TagDetector(AprilTags::tagCodes36h11)),has_camera_info_(false),tag_size_(0.2)
  {
    pub_detection_ = nh_p_.advertise<duckietown_msgs::AprilTags>("apriltags",1);
    string image_topic, camera_info_topic;
    //    nh_p_.getParam("image",image_topic);
    //    nh_p_.getParam("camera_info",camera_info_topic);
    sub_image_ = nh_p_.subscribe("/usb_cam/image_raw", 1, &AprilTagDetectorNode::cbImage, this);
    sub_camera_info_ = nh_p_.subscribe("/usb_cam/camera_info", 1, &AprilTagDetectorNode::cbCameraInfo, this);
  }

  ~AprilTagDetectorNode(){}
  void cbCameraInfo(const sensor_msgs::CameraInfo& camera_info_msg)
  {
    camera_info_ = camera_info_msg;
    has_camera_info_ = true;
  }
  void cbImage(const sensor_msgs::ImageConstPtr& image_msg){
    if (not has_camera_info_){
      ROS_INFO_STREAM("[AprilTagDetectorNode] Still waiting for CameraInfo.");
      return;
    }
    /* Convert image to cv::Mat */
    cv_bridge::CvImageConstPtr rgb_ptr = cv_bridge::toCvShare(image_msg);
    /* Convert to gray image*/
    cv::Mat image_gray;
    cv::cvtColor(rgb_ptr->image, image_gray, CV_BGR2GRAY);
    /* Detect april tag */
    vector<AprilTags::TagDetection> detections = tag_detector_.extractTags(image_gray);
    /* TODO convert to msg and publish */

    duckietown_msgs::AprilTags msg;
    for (int i = 0; i < detections.size(); ++i){
      msg.detections.push_back(toMsg(detections[i]));
    }
    msg.header.stamp = image_msg->header.stamp;
    /* TODO: Should use the id of image frame? or the vehicle frame? */
    msg.header.frame_id = image_msg->header.frame_id;
    pub_detection_.publish(msg);
  }
  /* Convert AprilTags::TagDetection to duckietown_msgs::TagDetection */
  duckietown_msgs::TagDetection toMsg (const AprilTags::TagDetection& detection)
  {
    duckietown_msgs::TagDetection msg;
    msg.good = detection.good;
    msg.id = detection.id;
    for (int i = 0; i < msg.p.size(); ++i){
      msg.p.push_back(detection.p[i].first);
      msg.p.push_back(detection.p[i].second);
    }
   
    msg.cxy.push_back(detection.cxy.first);
    msg.cxy.push_back(detection.cxy.second);
   
    msg.observedPerimeter = detection.observedPerimeter;

    msg.homography.push_back(detection.homography(0,0));
    msg.homography.push_back(detection.homography(0,1));
    msg.homography.push_back(detection.homography(0,2));
    msg.homography.push_back(detection.homography(1,0));
    msg.homography.push_back(detection.homography(1,1));
    msg.homography.push_back(detection.homography(1,2));
    msg.homography.push_back(detection.homography(2,0));
    msg.homography.push_back(detection.homography(2,1));
    msg.homography.push_back(detection.homography(2,2));

    msg.orientation = detection.getXYOrientation();

    msg.hxy.push_back(detection.hxy.first);
    msg.hxy.push_back(detection.hxy.second);

    /* Extract focal length and principal point */
    double fx = camera_info_.K[0];
    double fy = camera_info_.K[4];
    double px = camera_info_.K[2];
    double py = camera_info_.K[5];

    /* Extract relative translation rotation */
    Eigen::Vector3d trans;
    Eigen::Matrix3d rot;
    detection.getRelativeTranslationRotation(tag_size_,fx,fy,px,py,trans,rot);
    msg.transform.translation.x = trans(0);
    msg.transform.translation.y = trans(1);
    msg.transform.translation.z = trans(2);
    tf::Matrix3x3 tf_matrix3x3;
    tf::matrixEigenToTF(rot,tf_matrix3x3);
    tf::Quaternion quat;
    tf_matrix3x3.getRotation(quat);
    msg.transform.rotation.x = quat.x();
    msg.transform.rotation.y = quat.y();
    msg.transform.rotation.z = quat.z();
    msg.transform.rotation.w = quat.w();

    return msg;
  }
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "april_tag_detector");
  ros::NodeHandle nh("~");
  AprilTagDetectorNode node(nh);
  ros::spin();
  return 0;
}
