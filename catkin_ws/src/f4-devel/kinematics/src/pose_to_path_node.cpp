//
// Created by teddy on 3/17/16.
//
#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"
#include "nav_msgs/Path.h"
#include <string>
#include <ldap.h>

using namespace std;

class pose_to_path_node
{
public:
  pose_to_path_node(); // constructor

// class variables_
private:
  string node_name_;
  ros::NodeHandle nh_;
  string veh_name_;

  ros::Subscriber sub_pose_;
  ros::Publisher pub_path_;

  nav_msgs::Path path_;
  void poseCallback(geometry_msgs::PoseStampedConstPtr const& msg);
};

int main(int argc, char **argv){
  ros::init(argc, argv, "pose_to_path_node");
  pose_to_path_node node;
  ros::spin();
  return 0;
}

pose_to_path_node::pose_to_path_node() : nh_("~"), node_name_("pose_to_path_node")
{
  //Setup the publishers and subscirbers
  pub_path_ = nh_.advertise<nav_msgs::Path>("path", 10);
  sub_pose_ = nh_.subscribe("pose", 10, &pose_to_path_node::poseCallback, this);
  ROS_INFO_STREAM("[" << node_name_ << "] has started.");
}

void pose_to_path_node::poseCallback(geometry_msgs::PoseStampedConstPtr const& msg)
{
  path_.poses.push_back(*msg);
  path_.header = msg->header;
  pub_path_.publish(path_);
}

