#include "ros/ros.h"
#include "sensor_msgs/Image.h"
#include "sensor_msgs/CompressedImage.h"

#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

using namespace cv;
using namespace std;

void callback(const sensor_msgs::ImageConstPtr& imageMap)
{
    ROS_INFO("Received image %dx%d", imageMap->width, imageMap->height);
    cv_bridge::CvImagePtr cv_ptr;
    cv_ptr = cv_bridge::toCvCopy(imageMap, imageMap->encoding); 

    cv::Mat im_pi = cv::Mat::zeros(480, 640, CV_8UC3);
    im_pi = cv_ptr->image.clone();

}

void callback_compressed(const sensor_msgs::CompressedImageConstPtr& imageMap)
{
    ROS_INFO("Received Compressed Image ");
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "opencv_cpp_template");
    ros::NodeHandle n;

    ros::Subscriber sub = n.subscribe("/trabant/camera_node/image/compressed", 1000, callback_compressed);
    ros::spin();
    return 0;
}
