#include <iostream>
#include <ros/ros.h>
#include <stdlib.h>
#include <stdio.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
using namespace cv;
using namespace std;
Mat image;

void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
        image = cv_bridge::toCvCopy(msg, "bgr8")->image;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "image_listener");
    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);
    image_transport::Subscriber sub = 
        it.subscribe("/image_raw", 1, imageCallback);
    image_transport::Publisher  pub = it.advertise("echo_image", 1 );
    sensor_msgs::ImagePtr msg ;
    while (nh.ok()) {
        if (!image.empty()){
            msg = cv_bridge::CvImage(
                    std_msgs::Header(), "bgr8", image).toImageMsg();
            pub.publish(msg);
            }
        ros::spinOnce();
        }
  return 0;


 }

  
