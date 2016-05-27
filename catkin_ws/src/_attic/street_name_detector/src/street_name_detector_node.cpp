#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <duckietown_msgs/TagDetection.h>
#include <duckietown_msgs/AprilTags.h>

#include <opencv/cv.h>
#include <cv_bridge/cv_bridge.h>
#include <tf_conversions/tf_eigen.h>
#include <tf/transform_datatypes.h>

// #include "AprilTags/TagDetector.h"
// #include "AprilTags/Tag36h11.h"
#include "AprilTags/TagDetector.h"
#include "AprilTags/Tag36h11.h"

// For Street Name Detection
// MSER
#include "street_name_detector/region.h"
#include "street_name_detector/mser.h"

#include <opencv2/opencv.hpp>
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/features2d/features2d.hpp"

class StreetNameDetectorNode{
public:
  ros::NodeHandle nh_p_;
  ros::Publisher pub_detection_;
  ros::Publisher pub_image_;

  ros::Subscriber sub_image_;
  ros::Subscriber sub_camera_info_;

  AprilTags::TagDetector tag_detector_;
  sensor_msgs::CameraInfo camera_info_;
  bool has_camera_info_;
  double tag_size_; //Size of the tag in meters

  // variables for street name detector


  StreetNameDetectorNode(const ros::NodeHandle& nh): nh_p_(nh), tag_detector_(AprilTags::TagDetector(AprilTags::tagCodes36h11)),has_camera_info_(false),tag_size_(0.2)
  {
    pub_detection_ = nh_p_.advertise<duckietown_msgs::AprilTags>("apriltags",1);
    pub_image_     = nh_p_.advertise<sensor_msgs::Image>("tags_image",1); // remaps to /apriltags/...

    sub_image_ = nh_p_.subscribe("image_raw", 1, &StreetNameDetectorNode::cbImage, this); // remaps in launch file
    sub_camera_info_ = nh_p_.subscribe("camera_info", 1, &StreetNameDetectorNode::cbCameraInfo, this);


  }

  ~StreetNameDetectorNode(){}
  void cbCameraInfo(const sensor_msgs::CameraInfo& camera_info_msg)
  {
    camera_info_ = camera_info_msg;
    has_camera_info_ = true;
  }
  void cbImage(const sensor_msgs::ImageConstPtr& image_msg){
    if (not has_camera_info_){
      ROS_INFO_STREAM("[StreetNameDetectorNode] Still waiting for CameraInfo.");
      return;
    }
    /* Convert image to cv::Mat */
    cv_bridge::CvImageConstPtr rgb_ptr = cv_bridge::toCvShare(image_msg);
    /* Convert to gray image*/
    cv::Mat image = rgb_ptr->image;
    cv::Mat image_gray;
    cv::cvtColor(image, image_gray, CV_BGR2GRAY);

    /* Detect april tag */
    vector<AprilTags::TagDetection> detections = tag_detector_.extractTags(image_gray);
 
    /* Detect street name */
    //    cv::Mat im_in = image.clone();
    //    // pre-processing: keep green channel only for blobs
    //    cv::Mat im_channels[3];
    //    cv::split(image, im_channels);
    //    // set blue and red channels to 0
    //    im_channels[0]= cv::Mat::ones(image.rows, image.cols, CV_8UC1);
    //    im_channels[2]= cv::Mat::ones(image.rows, image.cols, CV_8UC1);
    //    im_channels[0] = 255 * im_channels[0];
    //    im_channels[2] = 255 * im_channels[2];
    //    //Merging green channel back to image
    //    cv::merge(im_channels, 3, image);

    vector<Region> regions_all, regions_black, regions_white, 
		regions_black_f, regions_white_f, regions_filtered;
    this->detect_regions(image, regions_black, regions_white);

    ROS_INFO_STREAM("Regions Black:\t" << regions_black.size());
    ROS_INFO_STREAM("Regions White:\t" << regions_white.size());

	// extract keypoints
	std::vector<cv::KeyPoint> kps1, kps2;
	// 1.1 detect FAST and SIFT in multi-scale
	kps1 = this->get_fast_corners(image, 3000);

    // try to draw something
    this->draw_regions(image, regions_black);
    cv::drawKeypoints(image, kps1, image);

    /* publish visualization */
    duckietown_msgs::AprilTags tags_msg;
    for (int i = 0; i < detections.size(); ++i){
      tags_msg.detections.push_back(toMsg(detections[i]));
      detections[i].draw(image);
    }
    tags_msg.header.stamp = image_msg->header.stamp;
    /* TODO: Should use the id of image frame? or the vehicle frame? */
    tags_msg.header.frame_id = image_msg->header.frame_id;

    pub_image_.publish(rgb_ptr->toImageMsg());

    pub_detection_.publish(tags_msg);
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

  // functions for street name detector
  void get_gradient_maps(cv::Mat& _grey_img,
  		cv::Mat& _gradient_magnitude, cv::Mat& _gradient_direction){

  	cv::Mat C = cv::Mat_<double>(_grey_img);

  	cv::Mat kernel = (cv::Mat_<double>(1,3) << -1,0,1);
  	cv::Mat grad_x;
  	filter2D(C, grad_x, -1, kernel, cv::Point(-1,-1), 0, cv::BORDER_DEFAULT);

  	cv::Mat kernel2 = (cv::Mat_<double>(3,1) << -1,0,1);
  	cv::Mat grad_y;
  	filter2D(C, grad_y, -1, kernel2, cv::Point(-1,-1), 0, cv::BORDER_DEFAULT);

  	for(int i=0; i<grad_x.rows; i++){
  		for(int j=0; j<grad_x.cols; j++){
  			_gradient_magnitude.at<double>(i,j) =
  					sqrt(pow(grad_x.at<double>(i,j),2)+pow(grad_y.at<double>(i,j),2));
  			_gradient_direction.at<double>(i,j) =
  					atan2(grad_y.at<double>(i,j), grad_x.at<double>(i,j));

  		}
  	}
  }

  void detect_regions(cv::Mat img,
  		vector<Region> & regions_black, vector<Region> & regions_white){

  	cv::Mat grey, lab_img, gradient_magnitude, gradient_direction;
  	gradient_magnitude = cv::Mat_<double>(img.size());
  	gradient_direction = cv::Mat_<double>(img.size());
  	this->get_gradient_maps( grey, gradient_magnitude, gradient_direction);
  	cvtColor(img, grey, CV_BGR2GRAY);
  	cvtColor(img, lab_img, CV_BGR2Lab);

  	// parameters:
  	// bool eight: 			[false]
  	// int delta			[25], smaller will get more regions
  	// double minArea,		[0.000008]
  	// double maxArea,		[0.03]
  	// double maxVariation	[1], smaller will get more regions
  	// double minDiversity	[0.7]
  	vector<Region> regions;
  	// for black regions,
  	::MSER mser_green(false, 25, 0.000008, 1.0, 1, 0.7);
  	// for white regions,
  	::MSER mser_white(false, 20, 0.000008, 0.03, 1, 0.7);

  	// try to detect green blob
  	mser_green((uchar*)grey.data, grey.cols, grey.rows, regions);
  	for (int i=0; i<regions.size(); i++){
  		regions[i].er_fill(grey);
  		regions[i].extract_features(lab_img, grey, gradient_magnitude);

  		// lab color model
  		//

  		if(regions[i].color_mean_[1] > 110
  				|| regions[i].color_mean_[0] > 150 //||
  				//regions[i].color_mean_[0] > 140
  				//regions[i].color_mean_[2] < 100
  		){
  			continue;
  		}

  		cv::Rect r = regions[i].bbox_;
  		float asp_ratio = (float)r.width / (float)r.height;

  		if(r.height < 20){
  			continue;
  		}
  		if(asp_ratio > 4 || asp_ratio < 1.5){
  			continue;
  		}

  		ROS_INFO_STREAM("r " << regions[i].color_mean_[0]);
  		regions_black.push_back(regions[i]);
  	}

  	//  	// step = 1: detect black
  	//  	// step = 2: detect white
  	//  	for (int step =1; step<2; step++)
  	//  	{
  	//
  	//  		if (step == 1){
  	//  			mser_black((uchar*)grey.data, grey.cols, grey.rows, regions);
  	//  		}
  	//  		if (step == 2){
  	//  			grey = 255-grey;
  	//  			mser_white((uchar*)grey.data, grey.cols, grey.rows, regions);
  	//  		}
  	//
  	//  		for (int i=0; i<regions.size(); i++)
  	//  			regions[i].er_fill(grey);
  	//
  	//  		double max_stroke = 0;
  	//  		for (int i=regions.size()-1; i>=0; i--)
  	//  		{
  	//  			regions[i].extract_features(lab_img, grey, gradient_magnitude);
  	//
  	//  			if ( (regions.at(i).stroke_std_/regions.at(i).stroke_mean_ > 0.8)
  	//  					|| (regions.at(i).num_holes_>2)
  	//  					|| (regions.at(i).bbox_.width <=3)
  	//  					|| (regions.at(i).bbox_.height <=3)
  	//  					)
  	//  				regions.erase(regions.begin()+i);
  	//  			else
  	//  				max_stroke = max(max_stroke, regions[i].stroke_mean_);
  	//  		}
  	//
  	//  		// filter
  	//  		for (int i=0; i<regions.size(); i++){
  	//  			cv::Rect r = regions[i].bbox_;
  	//  			float asp_ratio = (float)r.width / (float)r.height;
  	//  			float area_ratio = (float)regions[i].area_ / (float)r.area();
  	//
  	//  			if(r.width < 10){
  	//  				continue;
  	//  			}

  	//
  	//  			if(asp_ratio > 2){
  	//  				continue;
  	//  			}
  	//  			if(asp_ratio < 0.5){
  	//  				continue;
  	//  			}
  	//
  	//
  	//  			// stroke shouldn't be too small
  	//  	//		if(regions_all[i].stroke_mean_ < this->erMinStrokeWidth){
  	//  	//			continue;
  	//  	//		}
  	//  	//		cout << "intensity std: " << regions_in[i].intensity_std_ << endl;
  	//
  	//
  	//  			// black
  	//  			if(step == 1){
  	//  				if(area_ratio < 0.3){
  	//  					continue;
  	//  				}
  	//  				regions_black.push_back(regions[i]);
  	//  			}
  	//
  	//  			// white
  	//  			if(step == 2){
  	//  				if(area_ratio < 0.4){
  	//  					continue;
  	//  				}
  	//
  	//  				//cout << "area ratio: " << area_ratio << endl;
  	//  				regions_white.push_back(regions[i]);
  	//  			}
  	//  		}
  	//
  	//  		regions.clear();
  	//  	}

  }

  void filter_regions(
  		std::vector<Region> regions_black, std::vector<Region> regions_white,
  		std::vector<Region> & regions_out){

  	for(int i = 0; i < regions_black.size(); i++){

  		cv::Rect r1 = regions_black[i].bbox_; // child (black)
  		int is_placard = 0;

  		for(int j = 0; j < regions_white.size(); j++){

  			// check r1 is inside r2
  			cv::Rect r2 = regions_white[j].bbox_; // parent
  			cv::Rect r_i = r1 & r2;

  			//			if (r1.area() != r_i.area()){
  			//	continue;
  			//}

  			// check rize ratio, shouldn't be too small or large
  			//			float height_ratio = (float)r1.height / (float)r2.height;
  			//			if(height_ratio < 0.15 || height_ratio > 0.4){
  			//				continue;
  			//			}

  			// check black location; center of white should be inside black
  			//			cv::Point c_w = cv::Point(r2.x + 0.5*r2.width, r2.y + 0.5*r2.height);
  			//			if(!c_w.inside(r1)){
  			//				continue;
  			//			}

  			// intensity diff of child and parent should be high
  //			if(abs(regions_black[i].intensity_mean_ -
  //					regions_white[j].intensity_mean_) < this->erBlackOnWhiteVar){
  //				continue;
  //			}

  //			cout << "top left: (" << r1.x << ", " << r1.y << ")" << endl;
  //			cout << "r1 intensity mead: " <<regions_filtered[i].intensity_mean_ << endl;
  //			cout << "r1 intensity std : " <<regions_filtered[i].intensity_std_ << endl;
  //			cout << "r2 intensity mead: " <<regions_filtered[j].intensity_mean_ << endl;
  //			cout << "r2 intensity std : " <<regions_filtered[j].intensity_std_ << endl;
  //			cout << "perimeter_    : " <<regions_filtered[i].perimeter_ << endl;
  //			cout << "stroke_mean_  : " <<regions_filtered[i].stroke_mean_ << endl;
  //			cout << "pixel size    : " <<regions_filtered[i].pixels_.size() << endl;

  			is_placard = 1;
  			continue;
  		}
  		is_placard = 1;

  		if(is_placard == 1){
  			regions_out.push_back(regions_black[i]);
  			//cout << endl;
  		}

  	}
  }

  std::vector<cv::KeyPoint> get_fast_corners(cv::Mat im_gray, int kp_num){

  	std::vector<cv::KeyPoint> corners;

  	int threshold = 25;
  	bool nonmaxSuppression = true;

  	int pyramid_level = 1;

  	int maxTotalKeypoints = kp_num;
  	int gridRows=1;
  	int gridCols=1;

  	cv::Ptr<cv::FeatureDetector> detector(
  			new cv::DynamicAdaptedFeatureDetector(
  					new cv::FastAdjuster(threshold, nonmaxSuppression), 10, 100, 10));

  	cv::Ptr<cv::PyramidAdaptedFeatureDetector> pyramid_detector =
  			new cv::PyramidAdaptedFeatureDetector(detector, pyramid_level);

  	cv::Ptr<cv::GridAdaptedFeatureDetector> grid_detector =
  			new cv::GridAdaptedFeatureDetector(pyramid_detector,
  					maxTotalKeypoints, gridRows, gridCols);

  	grid_detector->detect(im_gray, corners);

  	return corners;
  }

  void draw_regions(cv::Mat &im_vis,
		  std::vector<Region> regions){

  	for(int i = 0; i < regions.size(); i++){
  		cv::rectangle(im_vis, regions[i].bbox_.tl(), regions[i].bbox_.br(), cv::Scalar(0, 255, 0), 3);
  	}

  }


};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "street_name_detector");
  ros::NodeHandle nh("~");
  StreetNameDetectorNode node(nh);
  ros::spin();
  return 0;
}
