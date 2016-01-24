#include <ros/ros.h>
#include <ros/package.h>
#include <fstream>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/CameraInfo.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <yaml-cpp/yaml.h>
#include <boost/filesystem.hpp>
#include "geometry_msgs/Point.h"
#include "ground_projection/GetGroundCoord.h"
#include "ground_projection/EstimateHomography.h"
#include "duckietown_msgs/Pixel.h"
#include "duckietown_msgs/Vector2D.h"
#include "duckietown_msgs/Segment.h"
#include "duckietown_msgs/SegmentList.h"

namespace enc = sensor_msgs::image_encodings;

#ifdef HAVE_NEW_YAMLCPP
template<typename T>
void operator >> (const YAML::Node& node, T& i)
{
  i = node.as<T>();
}
#endif

class GroundProjection
{
  ros::NodeHandle nh_;
  ros::ServiceServer service_homog_;
  ros::ServiceServer service_coord_;
  ros::Subscriber sub_lineseglist_;
  ros::Publisher pub_lineseglist_;
  sensor_msgs::CameraInfo::ConstPtr camera_info_;
  cv::Mat intrinsic_;
  cv::Mat distortion_;
  cv::Mat H_;
  bool rectified_input_;
  
public:
  GroundProjection()
  : nh_("~")
  {
    // load from homography yaml file
    std::string h_file;
    nh_.param<std::string>("homography_file", h_file, "package://ground_projection/homography/homography.yaml");
    h_file = get_package_filename(h_file);

    std::ifstream fin(h_file.c_str());
    if (!fin.good())
    {
      ROS_WARN_STREAM("Can't find homography file: " << h_file << " Using default calibration instead.");
      h_file = get_package_filename("package://ground_projection/homography/default.yaml");
    }

    ROS_INFO("load from homography yaml file [%s].", h_file.c_str());

    std::vector<float> h;
    h.resize(9);
    read_homography_yaml(h_file, h);

    // update homography matrix (H_)
    H_.create(3, 3, CV_32F);
    for(int r=0; r<3; r++)
    {
      for(int c=0; c<3; c++)
      {
        H_.at<float>(r, c) = h[r*3+c];
      }
    }

    nh_.param<bool>("rectified_input", rectified_input_, false);

    std::string camera_info_topic;
    nh_.param<std::string>("camera_info_path", camera_info_topic, "/camera_node/camera_info");
    ROS_INFO_STREAM("Waiting for message on topic " << camera_info_topic);
    camera_info_ = ros::topic::waitForMessage<sensor_msgs::CameraInfo>(camera_info_topic, nh_);

    if(!rectified_input_)
    {
      std::string distortion_model = camera_info_->distortion_model;
      if(distortion_model.compare(std::string("plumb_bob")) != 0)
        ROS_ERROR("Unexpected distortion_model: %s", distortion_model.c_str());
      
      intrinsic_.create(3, 3, CV_32F);
      for(int r=0; r<3; r++)
        for(int c=0; c<3; c++)
          intrinsic_.at<float>(r, c) = camera_info_->K[r*3+c];

      distortion_.create(1, 5, CV_32F);
      for(int i=0; i<5; i++)
        distortion_.at<float>(0, i) = camera_info_->D[i];
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
  duckietown_msgs::Pixel vector2pixel(duckietown_msgs::Vector2D& vec)
  {
    float w = static_cast<float>(camera_info_->width);
    float h = static_cast<float>(camera_info_->height);

    duckietown_msgs::Pixel pixel;
    
    pixel.u = static_cast<int>(w * vec.x);
    pixel.v = static_cast<int>(h * vec.y);

    // boundary check (might be redundant, but just in case)
    if(pixel.u < 0)     pixel.u = 0;
    if(pixel.u > w-1)   pixel.u = w-1;
    if(pixel.v < 0)     pixel.v = 0;
    if(pixel.v > h-1)   pixel.v = h-1;

    return pixel;
  }

  void estimate_ground_coordinate(duckietown_msgs::Vector2D& vec, geometry_msgs::Point& point)
  {
    duckietown_msgs::Pixel pixel = vector2pixel(vec);
    estimate_ground_coordinate(pixel, point);
  }

  void estimate_ground_coordinate(duckietown_msgs::Pixel& pixel, geometry_msgs::Point& point)
  {
    cv::Point3f pt_img(static_cast<float>(pixel.u), static_cast<float>(pixel.v), 1.f);

    if(!rectified_input_)
    {
      // undistort online
      std::vector<cv::Point2f> vpt_dist;
      vpt_dist.push_back(cv::Point2f(pt_img.x, pt_img.y));
      std::vector<cv::Point2f> vpt_undist;

      cv::undistortPoints(vpt_dist, vpt_undist, intrinsic_, distortion_, cv::Mat::eye(3, 3, CV_32F), intrinsic_);
      
      pt_img.x = vpt_undist[0].x;
      pt_img.y = vpt_undist[0].y;
    }
    
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
      estimate_ground_coordinate(msg_new.segments[i].pixels_normalized[0], msg_new.segments[i].points[0]);
      estimate_ground_coordinate(msg_new.segments[i].pixels_normalized[1], msg_new.segments[i].points[1]);
    }
    pub_lineseglist_.publish(msg_new);
  }

  bool get_ground_coordinate_cb(ground_projection::GetGroundCoord::Request &req, 
                                ground_projection::GetGroundCoord::Response &res)
  {
    estimate_ground_coordinate(req.normalized_uv, res.gp);

    ROS_INFO("image coord (u=%f, v=%f), ground coord (x=%f, y=%f, z=%f)", req.normalized_uv.x, req.normalized_uv.y, res.gp.x, res.gp.y, res.gp.z);
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

    // update internal homography
    H.copyTo(H_);

    // save to yaml file
    std::string h_file;
    nh_.param<std::string>("homography_file", h_file, "package://ground_projection/homography/homography.yaml");
    h_file = get_package_filename(h_file);

    ROS_INFO("save to homography yaml file [%s].", h_file.c_str());
    write_homography_yaml(h_file, res.homography);
    
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


  bool read_homography_yaml(const std::string& h_fname, 
                            std::vector<float>& h)
  {
    std::ifstream fin(h_fname.c_str());
    if (!fin.good())
    {
      ROS_INFO("Unable to open homography file [%s]", h_fname.c_str());
      return false;
    }
    // bool success = readCalibrationYml(fin, camera_name, cam_info);
    bool success = read_homography_yaml_(fin, h);

    if (!success)
      ROS_ERROR("Failed to parse homography from file [%s]", h_fname.c_str());
    return success;
  }

  bool read_homography_yaml_(std::istream& fin, std::vector<float>& h)
  {
    try
    {
#ifdef HAVE_NEW_YAMLCPP
      YAML::Node doc = YAML::Load(fin);
#else
      YAML::Parser parser(fin);
      if (!parser)
      {
        ROS_ERROR("Unable to create YAML parser for homography");
        return false;
      }
      YAML::Node doc;
      parser.GetNextDocument(doc);
#endif
      assert(h.size() == 9);

      const YAML::Node& h_data = doc["homography"];
      for(int i = 0; i < 9; ++i)
        h_data[i] >> h[i];

      return true;
    }    
    catch (YAML::Exception& e) {
      ROS_ERROR("Exception parsing YAML homography:\n%s", e.what());
      return false;
    }
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
