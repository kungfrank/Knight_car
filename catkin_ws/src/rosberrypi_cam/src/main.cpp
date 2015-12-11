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

using namespace raspicam;

boost::unordered_map<std::string, int> color_mode_map;

int main(int argc, char **argv) {
    color_mode_map["mono8"] = CV_8UC1;
    color_mode_map["rgb8"] = CV_8UC3;
    ros::init(argc, argv, "rosberrypi_cam");
    ros::NodeHandle nh("~");

    int fps;
    std::string color_mode;
    int height, width;
    nh.param("fps", fps, 10);
    nh.param<std::string>("color_mode", color_mode, "mono8");
    nh.param("height", height, 200);
    nh.param("width", width, 320);

    RaspiCam_Cv camera_cv;
    camera_cv.set(CV_CAP_PROP_FORMAT, CV_8UC1);
    camera_cv.set(CV_CAP_PROP_FORMAT, color_mode_map[color_mode]);
    camera_cv.set(CV_CAP_PROP_FPS, fps);
    camera_cv.set(CV_CAP_PROP_FRAME_WIDTH, width);
    camera_cv.set(CV_CAP_PROP_FRAME_HEIGHT, height);

    if(!camera_cv.open())
        ROS_ERROR("Error opening camera");
    sleep(3);
    camera_cv.grab();
    cv::Mat cv_img;

    std::string camera_info_url;
    nh.param<std::string>("camera_info_url", camera_info_url, "");
	
    image_transport::ImageTransport it(nh);
    image_transport::CameraPublisher pub = it.advertiseCamera("image_raw", 1);

    std::string camera_name = nh.getNamespace();
    camera_info_manager::CameraInfoManager cinfo_(nh, camera_name);

    ros::Rate rate(fps);
    while(ros::ok()) {
        //printf("- %f... ", ros::Time::now().toSec());
        camera_cv.grab();
        camera_cv.retrieve(cv_img);
        std_msgs::Header header();
        cv_bridge::CvImage imgmsg;
        sensor_msgs::CameraInfo ci = cinfo_.getCameraInfo();
        imgmsg.header.frame_id = camera_name + "_optical_frame";
        ci.header.frame_id = imgmsg.header.frame_id;
        imgmsg.encoding = color_mode;
        imgmsg.image = cv_img;
        pub.publish(*imgmsg.toImageMsg(), ci, ros::Time::now());
        //printf("%f\n", ros::Time::now().toSec());
        rate.sleep();
        ros::spinOnce();
    }
    return 0;
}
