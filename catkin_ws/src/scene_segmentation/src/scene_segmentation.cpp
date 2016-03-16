#include <ros/ros.h>
#include <ros/package.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/CameraInfo.h>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <time.h>
#include "egbis.h"
#include "egbis/segment-image.h"
#include "egbis/misc.h"
#include "egbis/image.h"
#include <boost/shared_ptr.hpp>
#include "scene_segmentation/SegmentImage.h"
#include "duckietown_msgs/Rect.h"
#include "duckietown_msgs/Rects.h"
#include "duckietown_msgs/SceneSegments.h"

namespace enc = sensor_msgs::image_encodings;

// Graph-based Image Segmentation wrapper
class Segmentor
{
  int width_, height_;
  image<rgb>* image_;
  cv::Mat segimage_;
  cv::Mat segimage_gray_;
  std::vector<rect> region_rects_;
  bool draw_rect_;
public:
  Segmentor(int width, int height, bool draw_rect)
  : width_(width), height_(height)
  {
    image_ = new image<rgb>(width_, height_);
    segimage_ = cv::Mat(cv::Size(width_, height_), CV_8UC3);
    segimage_gray_ = cv::Mat(cv::Size(width_, height_), CV_8UC1);
    draw_rect_ = draw_rect;
  }

  ~Segmentor()
  {
    delete image_;
  }

  typedef typename boost::shared_ptr<Segmentor> Ptr;
  typedef typename boost::shared_ptr<const Segmentor> ConstPtr;

  cv::Mat& segment(const cv::Mat& input, float sigma, float k, int min_size, int *numccs, std::vector<duckietown_msgs::Rect>& rects)
  {
    int w = input.cols;
    int h = input.rows;

    convert_mat2image(input, image_);

    image<rgb> *segmentedImage = segment_image(image_, sigma, k, min_size, numccs, region_rects_);

    convert_image2mat(segmentedImage, segimage_);
    cv::cvtColor(segimage_, segimage_gray_, CV_BGR2GRAY);
    cv::cvtColor(segimage_gray_, segimage_, CV_GRAY2BGR);
    
    rects.resize(region_rects_.size());
    for(int i=0; i<region_rects_.size(); i++)
    {
      ROS_DEBUG_STREAM("[" << i << "]" << region_rects_[i].x << " " << region_rects_[i].y << " " << region_rects_[i].w << " " << region_rects_[i].h);

      // draw green rectangles
      if(draw_rect_)
      {
        cv::rectangle(segimage_, 
          cv::Point(region_rects_[i].x, region_rects_[i].y), 
          cv::Point(region_rects_[i].x + region_rects_[i].w, region_rects_[i].y + region_rects_[i].h), 
          cv::Scalar(0, 255, 0), 1, 8);
      }
      rects[i].x = int32_t(region_rects_[i].x);
      rects[i].y = int32_t(region_rects_[i].y);
      rects[i].w = int32_t(region_rects_[i].w);
      rects[i].h = int32_t(region_rects_[i].h);
    }
    delete segmentedImage;
    return segimage_;
  }

private:
  void convert_mat2image(const cv::Mat& cvmat, image<rgb>* image)
  {
    int w = cvmat.cols;
    int h = cvmat.rows;
    assert(width_ == w);
    assert(height_ == h);
    std::memcpy(image->data, cvmat.data, sizeof(unsigned char)*3*w*h);
  }

  void convert_image2mat(const image<rgb>* image, cv::Mat& cvmat)
  {
    int w = image->width();
    int h = image->height();
    assert(width_ == w);
    assert(height_ == h);
    std::memcpy(cvmat.data, image->data, sizeof(unsigned char)*3*w*h);
  }
};

class SceneSegmentation
{
  ros::NodeHandle nh_;
  ros::ServiceServer srv_segment_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber sub_image_;
  image_transport::Publisher pub_image_;
  ros::Publisher pub_rects_;
  ros::Publisher pub_segments_;
  bool half_;
  
  Segmentor::Ptr segmentor_;
  sensor_msgs::CameraInfo::ConstPtr camera_info_;
  
  std::vector<duckietown_msgs::Rect> rects_;
  duckietown_msgs::Rects rects_msg_;
  duckietown_msgs::SceneSegments segments_msg_;

  // params for segmentation
  //
  // Efficient Graph-Based Image Segmentation
  // P. Felzenszwalb, D. Huttenlocher
  // International Journal of Computer Vision, Vol. 59, No. 2, September 2004
  float sigma_;
  float k_;
  int min_;

public:
  SceneSegmentation()
  : nh_("~"), it_(nh_)
  {
    nh_.param<float>("sigma", sigma_, 0.5f);
    nh_.param<float>("k", k_, 500.f);
    nh_.param<int>("min", min_, 20);

    nh_.param<bool>("half", half_, true);
    bool draw_rect;
    nh_.param<bool>("draw_rect", draw_rect, true);

    ROS_INFO("Waiting for message on camera info topic.");
    camera_info_ = ros::topic::waitForMessage<sensor_msgs::CameraInfo>("camera_info", nh_);

    // segmentor_ = Segmentor::Ptr(new Segmentor(camera_info_->width, camera_info_->height, half_));
    if(half_)
      segmentor_ = Segmentor::Ptr(new Segmentor(camera_info_->width/2, camera_info_->height/2, draw_rect));
    else
      segmentor_ = Segmentor::Ptr(new Segmentor(camera_info_->width, camera_info_->height, draw_rect));

    srv_segment_ = nh_.advertiseService("segment_image", &SceneSegmentation::segment_image_cb, this);
    ROS_INFO("segment_image is ready.");

    sub_image_ = it_.subscribe("image_in", 1, &SceneSegmentation::image_cb, this);
    pub_image_ = it_.advertise("image_out", 1);
    pub_rects_ = nh_.advertise<duckietown_msgs::Rects>("rects_out", 1);
    pub_segments_ = nh_.advertise<duckietown_msgs::SceneSegments>("segments_out", 1);
  }

  ~SceneSegmentation()
  {

  }

private:
  void segment(const cv::Mat& image, cv::Mat& segimage, std::vector<duckietown_msgs::Rect>& rects)
  {
    int num_seg;
    clock_t t;
    t = clock();
    segimage = segmentor_->segment(image, sigma_, k_, min_, &num_seg, rects);
    t = clock() - t;
    ROS_INFO("segment takes %f ms.", float(t)/CLOCKS_PER_SEC*1000.f);
    ROS_INFO("%d segments are found.", num_seg);
  }

  void image_cb(const sensor_msgs::Image::ConstPtr& image)
  {
    cv_bridge::CvImageConstPtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvShare(image, enc::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

    cv_bridge::CvImagePtr seg_ptr (new cv_bridge::CvImage);

    seg_ptr->header = cv_ptr->header;
    seg_ptr->encoding = cv_ptr->encoding;

    if(half_)
    {
      // shrink & expand
      cv::Size half_size = cv::Size(cv_ptr->image.cols/2, cv_ptr->image.rows/2);
      cv::Mat half_img = cv::Mat(half_size, CV_8UC3);
      cv::Mat half_seg = cv::Mat(half_size, CV_8UC3);
      cv::resize(cv_ptr->image, half_img, half_size);

      segment(half_img, half_seg, rects_);

      cv::Size full_size = cv::Size(cv_ptr->image.cols, cv_ptr->image.rows);
      cv::resize(half_seg, seg_ptr->image, full_size);

      // expand `rects`
      for(int i=0; i<rects_.size(); i++)
      {
        rects_[i].x *= 2;
        rects_[i].y *= 2;
        rects_[i].w *= 2;
        rects_[i].h *= 2;
      }
    }
    else
      segment(cv_ptr->image, seg_ptr->image, rects_);

    pub_image_.publish(seg_ptr->toImageMsg());

    rects_msg_.rects = rects_;
    pub_rects_.publish(rects_msg_);

    segments_msg_.segimage = *seg_ptr->toImageMsg();
    segments_msg_.rects = rects_;
    pub_segments_.publish(segments_msg_);
  }

  bool segment_image_cb(scene_segmentation::SegmentImage::Request &req, 
                        scene_segmentation::SegmentImage::Response &res)
  {
    cv_bridge::CvImagePtr cv_ptr;

    try
    {
      cv_ptr = cv_bridge::toCvCopy(req.image, enc::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return false;
    }

    cv_bridge::CvImagePtr seg_ptr;
    seg_ptr->header = cv_ptr->header;
    seg_ptr->encoding = cv_ptr->encoding;

    segment(cv_ptr->image, seg_ptr->image, rects_);
    
    res.ss.segimage = *seg_ptr->toImageMsg();
    res.ss.rects = rects_;
    return true;
  }
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "scene_segmentation");

  SceneSegmentation ss;

  ros::spin();

  return 0;
}
