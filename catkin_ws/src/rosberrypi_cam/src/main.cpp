#include <raspicam/raspicam.h>
#include <raspicam/raspicam_cv.h>
#include <ros/ros.h>
#include <ros/console.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/CameraInfo.h>
#include <std_msgs/Header.h>
#include <camera_info_manager/camera_info_manager.h>
#include <boost/unordered_map.hpp>
#include <iostream>

using namespace raspicam;

boost::unordered_map<std::string, int> color_mode_map;

int main(int argc, char **argv) {
    // color_mode_map["mono8"] = CV_8UC1;
    // color_mode_map["rgb8"] = CV_8UC3;
    ros::init(argc, argv, "rosberrypi_cam");
    ros::NodeHandle nh("~");

    int fps;
    std::string color_mode = "bgr8"; // "mono8" for grey 
    int height, width;
    int brightness, contrast, saturation, gain, exposure, white_balance_red_v, white_balance_blue_u;

    nh.param("fps", fps, 60);
    // nh.param<std::string>("color_mode", color_mode, "mono8");
    nh.param("height", height, 200);
    nh.param("width", width, 320);
    
    nh.param("brightness", brightness, 50);
    nh.param("contrast", contrast, 50);
    nh.param("saturation", saturation, 50);
    nh.param("gain", gain, 50);
    nh.param("exposure", exposure, 50);
    nh.param("white_balance_red_v", white_balance_red_v, 50);
    nh.param("white_balance_blue_u", white_balance_blue_u, 50);

    RaspiCam_Cv camera_cv;
    camera_cv.set(CV_CAP_PROP_FORMAT, CV_8UC3);
    camera_cv.set(CV_CAP_PROP_FPS, fps);
    camera_cv.set(CV_CAP_PROP_FRAME_WIDTH, width);
    camera_cv.set(CV_CAP_PROP_FRAME_HEIGHT, height);
    camera_cv.set(CV_CAP_PROP_BRIGHTNESS, 50);
    camera_cv.set(CV_CAP_PROP_CONTRAST, 50);
    camera_cv.set(CV_CAP_PROP_SATURATION, 50);
    camera_cv.set(CV_CAP_PROP_GAIN, 50);
    camera_cv.set(CV_CAP_PROP_EXPOSURE, 50);
    camera_cv.set(CV_CAP_PROP_WHITE_BALANCE_RED_V, 50);
    camera_cv.set(CV_CAP_PROP_WHITE_BALANCE_BLUE_U, 50);

    if(!camera_cv.open())
        ROS_ERROR("Error opening camera");
    sleep(3);
    camera_cv.grab();
    cv::Mat cv_img;

    std::string camera_info_url;
    nh.param<std::string>("camera_info_url", camera_info_url, "package://rosberrypi_cam/calibration/picamera.yaml");
    
    image_transport::ImageTransport it(nh);
    image_transport::CameraPublisher pub = it.advertiseCamera("image_raw", 1);

    // std::string camera_name = nh.getNamespace();
    std::string camera_name = ros::this_node::getName();
    ROS_INFO_STREAM("[rosberrypi_cam] Camera Name:" << camera_name);
    camera_info_manager::CameraInfoManager cinfo_(nh, camera_name);
    cinfo_.loadCameraInfo(camera_info_url);

    ros::Rate rate(fps);
    while(ros::ok()) {
        // printf("- %f... ", ros::Time::now().toSec());
        camera_cv.grab();
        camera_cv.retrieve(cv_img);
	// printf("done\n");
        std_msgs::Header header();

        cv_bridge::CvImage imgmsg;
        sensor_msgs::CameraInfo ci = cinfo_.getCameraInfo();
        imgmsg.header.frame_id = camera_name + "_optical_frame";
        ci.header.frame_id = imgmsg.header.frame_id;
        imgmsg.encoding = color_mode;
        // imgmsg.encoding = "bgr8";
        imgmsg.image = cv_img;
        pub.publish(*imgmsg.toImageMsg(), ci, ros::Time::now());
        //printf("%f\n", ros::Time::now().toSec());
        rate.sleep();
        ros::spinOnce();
    }
    return 0;
}
