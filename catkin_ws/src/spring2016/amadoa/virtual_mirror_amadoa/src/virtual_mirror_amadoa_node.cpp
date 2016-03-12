#include "ros/ros.h"
#include "std_msgs/String.h"
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
/*
  try
  {
    cv::imshow("view", cv_bridge::toCvShare(msg, "bgr8")->image);
    cv::waitKey(30);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
  }
*/
// TODO process and publish mirrored image

//  ROS_INFO("msg.height: [%s]", msg.height);
  ROS_INFO("imageCallback called");
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "virtual_mirror_amadoa_node");

  ros::NodeHandle nh;
  image_transport::ImageTransport it(nh);
  
  image_transport::Subscriber sub = it.subscribe("/amadobot/camera_node/image", 1000, imageCallback);
  ROS_INFO("image subscriber created");


 // image_transport::Publisher pub = it.advertise("/amadobot/virtual_mirror_amado_node/image/mirrored", 1);

  ros::Rate loop_rate(10);

  while (ros::ok())
  {
    ros::spinOnce();

    loop_rate.sleep();
  }

  return 0;
}
