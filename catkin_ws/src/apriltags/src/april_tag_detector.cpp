#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
// #include <sensor_msgs/CompressedImage.h>
#include <cv_bridge/cv_bridge.h>

#include <opencv/cv.h>
#include "AprilTags/TagDetector.h"
#include "AprilTags/Tag36h11.h"

class AprilTagDetectorNode{
public:
  ros::NodeHandle nh_p_;
  ros::Subscriber sub_image_;
  ros::Subscriber sub_camera_info_;

  AprilTags::TagDetector tag_detector_;
  sensor_msgs::CameraInfo camera_info_;
  bool has_camera_info_;

  AprilTagDetectorNode(const ros::NodeHandle& nh): nh_p_(nh), tag_detector_(AprilTags::TagDetector(AprilTags::tagCodes36h11)),has_camera_info_(false)
  {
    sub_image_ = nh_p_.subscribe("image", 1, &AprilTagDetectorNode::cbImage, this);
    sub_camera_info_ = nh_p_.subscribe("camera_info", 1, &AprilTagDetectorNode::cbCameraInfo, this);
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