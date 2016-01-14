#include <ros/ros.h>
#include <ros/package.h>
#include <fstream>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <yaml-cpp/yaml.h>
#include <boost/filesystem.hpp>
#include "ground_projection/GetGroundCoord.h"
#include "ground_projection/EstimateHomography.h"
#include "duckietown_msgs/Pixel.h"
#include "geometry_msgs/Point.h"
#include "duckietown_msgs/Segment.h"
#include "duckietown_msgs/SegmentList.h"

namespace enc = sensor_msgs::image_encodings;

class GroundProjection
{
  ros::NodeHandle nh_;
  ros::ServiceServer service_homog_;
  ros::ServiceServer service_coord_;
  ros::Subscriber sub_lineseglist_;
  ros::Publisher pub_lineseglist_;
  cv::Mat H_;
  
public:
  GroundProjection()
  : nh_("~")
  {
    // load homography matrix from rosparam
    std::vector<float> h;
    nh_.getParam("homography", h);
    H_.create(3, 3, CV_32F);
    for(int r=0; r<3; r++)
    {
      for(int c=0; c<3; c++)
      {
        H_.at<float>(r, c) = h[r*3+c];
      }
    }

    service_homog_ = nh_.advertiseService("estimate_homography", &GroundProjection::estimate_homography_cb, this);
    ROS_INFO("estimate_homography is ready.");

    service_coord_ = nh_.advertiseService("get_ground_coordinate", &GroundProjection::get_ground_coordinate_cb, this);
    ROS_INFO("get_ground_coordinate is ready.");

    sub_lineseglist_ = nh_.subscribe("lineseglist_in", 1, &GroundProjection::lineseglist_cb, this);

    pub_lineseglist_ = nh_.advertise<duckietown_msgs::SegmentList>("lineseglist_out", 1);
  }

  ~GroundProjection()
  {
  }

private:
  void estimate_ground_coordinate(duckietown_msgs::Pixel& pixel, geometry_msgs::Point& point)
  {
    cv::Point3f pt_img (float(pixel.u), float(pixel.v), 1.f);
    // cv::Point3f pt_img (float(pixel.u*2), float(pixel.v*2), 1.f);
    cv::Mat pt_gnd_ = H_ * cv::Mat(pt_img);
    cv::Point3f pt_gnd(pt_gnd_);

    point.x = pt_gnd.x/pt_gnd.z;
    point.y = pt_gnd.y/pt_gnd.z;
    point.z = 0.f;
  }

  void lineseglist_cb(const duckietown_msgs::SegmentList::ConstPtr& msg)
  {
    duckietown_msgs::SegmentList msg_new = *msg;
    for(int i=0; i<msg_new.segments.size(); i++)
    {
      // each line segment is composed of two end points [0:1]
      estimate_ground_coordinate(msg_new.segments[i].pixels[0], msg_new.segments[i].points[0]);
      estimate_ground_coordinate(msg_new.segments[i].pixels[1], msg_new.segments[i].points[1]);
    }
    pub_lineseglist_.publish(msg_new);
  }

  bool get_ground_coordinate_cb(ground_projection::GetGroundCoord::Request &req, 
                                ground_projection::GetGroundCoord::Response &res)
  {
    estimate_ground_coordinate(req.uv, res.gp);

    ROS_INFO("image coord (u=%d, v=%d), ground coord (x=%f, y=%f, z=%f)", req.uv.u, req.uv.v, res.gp.x, res.gp.y, res.gp.z);
    return true;
  }

  bool estimate_homography_cb(ground_projection::EstimateHomography::Request &req, 
                              ground_projection::EstimateHomography::Response &res)
  {
    cv_bridge::CvImagePtr cv_ptr;

    try
    {
      if(enc::isColor(req.image.encoding))
        cv_ptr = cv_bridge::toCvCopy(req.image, enc::BGR8);
      else
        cv_ptr = cv_bridge::toCvCopy(req.image, enc::MONO8);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return false;
    }

    cv::Mat H;
    if(!estimate_homography(cv_ptr->image, H))
    {
      return false;
    }

    // prepare response
    res.homography.resize(9);
    for(int r=0; r<3; r++)
    {
      for(int c=0; c<3; c++)
      {
        res.homography[r*3+c] = H.at<float>(r, c);
      }
    }

    // update to rosparam
    nh_.setParam("homography", res.homography);
    ROS_INFO("update homography rosparam.");

    // update internal homography
    H.copyTo(H_);

    // save to yaml file
    std::string homography_file;
    nh_.param<std::string>("homography_file", homography_file, "package://ground_projection/homography.yaml");
    homography_file = get_package_filename(homography_file);

    ROS_INFO("save to homography yaml file [%s].", homography_file.c_str());
    write_homography_yaml(homography_file, res.homography);
    
    return true;
  }

  bool estimate_homography(cv::Mat& Ir, cv::Mat& H)
  {
    // estimate homography
    int board_w, board_h;
    float square_size;
    nh_.param<int>("board_w", board_w, 7);
    nh_.param<int>("board_h", board_h, 5);
    nh_.param<float>("square_size", square_size, 0.031f);

    int board_n = board_w * board_h;

    cv::Size board_size(board_w, board_h);
    std::vector<cv::Point2f> corners_;
    bool found = findChessboardCorners(Ir, board_size, corners_);
    if(found)
    {
      cornerSubPix(Ir, corners_, cv::Size(11, 11), cv::Size(-1, -1),
        cv::TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 30, 0.1));
    }
    else
    {
      ROS_ERROR("Cound't find a checkerboard");
      return false;
    }

    std::vector<cv::Point2f> pts_gnd_, pts_img_;
    pts_gnd_.resize(board_w*board_h);
    pts_img_.resize(board_w*board_h);

    float x_offset, y_offset;
    nh_.param<float>("x_offset", x_offset, 0.191f);
    nh_.param<float>("y_offset", y_offset, -0.093f);
    cv::Point2f offset = cv::Point2f(x_offset, y_offset);

    for(int r=0; r<board_h; r++)
    {
      for(int c=0; c<board_w; c++)
      {
        pts_gnd_[r*(board_w)+c] = cv::Point2f(float(r)*square_size, float(c)*square_size) + offset;
        pts_img_[r*(board_w)+c] = corners_[r*(board_w)+c];
      }
    }

    H = cv::findHomography(pts_img_, pts_gnd_, CV_RANSAC);
    H.convertTo(H, CV_32F);

    return true;
  }

  bool write_homography_yaml(const std::string& h_fname,
                             const std::vector<float>& h)
  {
    boost::filesystem::path dir(boost::filesystem::path(h_fname).parent_path());
    if(!dir.empty() && !boost::filesystem::exists(dir) &&
      !boost::filesystem::create_directories(dir))
    {
      ROS_ERROR("Unable to create directory for homography file [%s]", dir.c_str());
    }

    std::ofstream out(h_fname.c_str());
    if(!out.is_open())
    {
      ROS_ERROR("Unable to open homography file [%s] for writing", h_fname.c_str());
      return false;
    }

    YAML::Emitter emitter;
    emitter << YAML::BeginMap;

    emitter << YAML::Key << "homography" << YAML::Value << YAML::Flow;

    emitter << YAML::BeginSeq;
    for (int i = 0; i < h.size(); i++)
      emitter << h[i];
    emitter << YAML::EndSeq;

    emitter << YAML::EndMap;

    out << emitter.c_str();
    return true;
  }

  
  std::string get_package_filename(const std::string &url)
  {
    ROS_DEBUG_STREAM("homography URL: " << url);

    // Scan URL from after "package://" until next '/' and extract
    // package name.
    size_t prefix_len = std::string("package://").length();
    size_t rest = url.find('/', prefix_len);
    std::string package(url.substr(prefix_len, rest - prefix_len));

    // Look up the ROS package path name.
    std::string pkgPath(ros::package::getPath(package));
    if(pkgPath.empty())
    {
      ROS_WARN_STREAM("unknown package: " << package << " (ignored)");
      return pkgPath;
    }
    else
    {
      // Construct file name from package location and remainder of URL.
      return pkgPath + url.substr(rest);
    }
  }

};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "ground_projection");

  GroundProjection gp;

  ros::spin();

  return 0;
}
