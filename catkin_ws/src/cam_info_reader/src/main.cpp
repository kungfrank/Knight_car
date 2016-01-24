#include <ros/ros.h>
#include <ros/console.h>
#include <sensor_msgs/CameraInfo.h>
#include <std_msgs/Header.h>
#include <camera_info_manager/camera_info_manager.h>
#include <iostream>


int main(int argc, char **argv) {
    ros::init(argc, argv, "cam_info_reader");
    ros::NodeHandle nh("~");

    int hz;
    nh.param("hz", hz, 1);

    std::string camera_info_url;
    nh.param<std::string>("camera_info_url", camera_info_url, "package://duckietown/config/baseline/calibration/camera_intrinsic/default.yaml");    
    ros::Publisher pub = nh.advertise<sensor_msgs::CameraInfo>("camera_info",1);

    std::string camera_name = ros::this_node::getName();
    ROS_INFO_STREAM("[cam_info_reader] Camera Name: " << camera_name);
    ROS_INFO_STREAM("[cam_info_reader] Cam_info_url: " << camera_info_url);
    camera_info_manager::CameraInfoManager cinfo_(nh, camera_name);
    cinfo_.loadCameraInfo(camera_info_url);

    bool published = false;

    ros::Rate rate(hz);
    while(ros::ok()) {
        std_msgs::Header header();
        sensor_msgs::CameraInfo ci = cinfo_.getCameraInfo();
        pub.publish(ci);

        if (not published){
            ROS_INFO_STREAM("[cam_info_reader] Published the camera info at least once.");
            published = true;
        }

        rate.sleep();
        ros::spinOnce();
    }
    return 0;
}
