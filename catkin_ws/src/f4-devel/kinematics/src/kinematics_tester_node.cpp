//
// Created by teddy on 3/17/16.
//
#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "duckietown_msgs/Twist2DStamped.h"
#include <string>

using namespace std;

class kinematics_tester_node
{
public:
  kinematics_tester_node(); // constructor

// class variables_
private:
  string node_name_;
  ros::NodeHandle nh_;
  string veh_name_;

  ros::Publisher pub_carCmdOut_;
  ros::Subscriber sub_twistIn_;

  //Constant velocity arguments
  float v_, omega_, hz_;
  void twistCallback(geometry_msgs::TwistConstPtr const& msg);
};

int main(int argc, char **argv){
  ros::init(argc, argv, "kinematics_tester_node");
  kinematics_tester_node node;
  ros::spin();
  return 0;
}

kinematics_tester_node::kinematics_tester_node() : nh_("~"), node_name_("kinematics_tester_node")
{
  //Setup the publishers and subscirbers
  pub_carCmdOut_ = nh_.advertise<duckietown_msgs::Twist2DStamped>("car_cmd", 10);

  //Setup parameters
  nh_.param("v", v_, 0.0f);
  nh_.param("omega", omega_, 0.0f);
  nh_.param("hz", hz_, 0.0f);

  ROS_INFO_STREAM("[" << node_name_ << "] has started.");

  //If parameters were given, publish them
  if(hz_ > 0)
  {
    duckietown_msgs::Twist2DStamped cmd_msg;

    cmd_msg.v = v_;
    cmd_msg.omega = omega_;
    ros::Rate rate(hz_);
    while(ros::ok()){
      cmd_msg.header.stamp = ros::Time::now();
      pub_carCmdOut_.publish(cmd_msg);
      rate.sleep();
    }
  }
  else
  {  
    sub_twistIn_ = nh_.subscribe("twist_in", 10, &kinematics_tester_node::twistCallback, this);
  }
}

void kinematics_tester_node::twistCallback(geometry_msgs::TwistConstPtr const& msg)
{
  //Grab the 2D parts of the twist and stuff it into a duckietown Twist2dStamed and publish
  duckietown_msgs::Twist2DStamped cmd_msg;
  cmd_msg.header.stamp = ros::Time::now(); //TODO check if the robot publisher publishes time
  cmd_msg.v = msg->linear.x;
  cmd_msg.omega = msg->angular.z;
  pub_carCmdOut_.publish(cmd_msg);
}

