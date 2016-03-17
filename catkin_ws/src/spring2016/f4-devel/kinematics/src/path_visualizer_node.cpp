//
// Created by teddy on 3/17/16.
//
#include "ros/ros.h"
#include "duckietown_msgs/Pose2DStamped.h"
#include "nav_msgs/Path.h"
#include "geometry_msgs/PointStamped.h"
#include "tf/transform_datatypes.h"
#include <string>
#include <ldap.h>
#include <tf/LinearMath/Quaternion.h>

using namespace std;

class path_visualizer_node
{
public:
  path_visualizer_node(); // constructor

// class variables_
private:
  string node_name_;
  ros::NodeHandle nh_;
  string veh_name_;

  ros::Subscriber sub_pose_;
  ros::Publisher pub_path_;

  nav_msgs::Path path_;
  void poseCallback(duckietown_msgs::Pose2DStampedConstPtr const& msg);
};

int main(int argc, char **argv){
  ros::init(argc, argv, "path_visualizer_node");
  path_visualizer_node node;
  ros::spin();
  return 0;
}

path_visualizer_node::path_visualizer_node() : nh_("~"), node_name_("path_visualizer_node")
{
  //Setup the publishers and subscirbers
  pub_path_ = nh_.advertise<nav_msgs::Path>("path", 10);
  sub_pose_ = nh_.subscribe("pose", 10, &path_visualizer_node::poseCallback, this);
  ROS_INFO_STREAM("[" << node_name_ << "] has started.");
}

void path_visualizer_node::poseCallback(duckietown_msgs::Pose2DStampedConstPtr const& msg)
{
  geometry_msgs::PoseStamped pose_msg;
  pose_msg.header = msg->header;
  pose_msg.pose.position.x = msg->x;
  pose_msg.pose.position.y = msg->y;
  pose_msg.pose.orientation = tf::createQuaternionMsgFromYaw(msg->theta);
  path_.poses.push_back(pose_msg);
  path_.header = msg->header;
  pub_path_.publish(path_);
}

