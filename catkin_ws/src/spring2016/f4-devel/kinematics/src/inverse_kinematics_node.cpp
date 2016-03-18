//
// Created by teddy on 3/17/16.
//
#include "ros/ros.h"
#include <string>
#include <duckietown_msgs/Twist2DStamped.h>
#include <duckietown_msgs/WheelsCmdStamped.h>

using namespace std;

class inverse_kinematics_node
{
public:
  inverse_kinematics_node(); // constructor

// class variables_
private:
  string node_name_;
  ros::NodeHandle nh_;
  string veh_name_;

  ros::Subscriber sub_twist_;
  ros::Publisher pub_wheelsCmd_;

  double K_l_; // duty to left wheel rotation rate constant
  double K_r_; // duty to right wheel rotation rate constant
  double radius_l_; // radius of the left wheel
  double radius_r_; // radius of the right wheel
  double baseline_lr_; //distance between the center of the two wheels

  void carCmdCallback(duckietown_msgs::Twist2DStampedConstPtr const& msg);
};

int main(int argc, char **argv){
  ros::init(argc, argv, "inverse_kinematics_node");
  inverse_kinematics_node node;
  ros::spin();
  return 0;
}

inverse_kinematics_node::inverse_kinematics_node(): nh_("~"), node_name_("inverse_kinematics_node")
{
  //Setup the publishers and subscirbers
  pub_wheelsCmd_ = nh_.advertise<duckietown_msgs::WheelsCmdStamped>("wheels_cmd", 10);
  sub_twist_ = nh_.subscribe("car_cmd", 10, &inverse_kinematics_node::carCmdCallback, this);

  //Get parameters
  nh_.param("K_l", K_l_, 25.0);
  nh_.param("K_r", K_r_, 25.0);
  nh_.param("radius_l", radius_l_, 0.02);
  nh_.param("radius_r", radius_r_, 0.02);
  nh_.param("baseline_lr", baseline_lr_, 0.1);

  ROS_INFO_STREAM("[" << node_name_ << "] has started.");
}

void inverse_kinematics_node::carCmdCallback(duckietown_msgs::Twist2DStampedConstPtr const& msg)
{
  // conversion from linear and angular velocities to motor rotation rate
  double omega_r = (msg->v + 0.5 * msg->omega * baseline_lr_)/radius_r_;
  double omega_l = (msg->v - 0.5 * msg->omega * baseline_lr_)/radius_l_;

  // conversion from motor rotation rate to duty cycle
  double u_r = omega_r / K_r_;
  double u_l = omega_l / K_l_;

  // stuff the wheel commands in a message and publish
  duckietown_msgs::WheelsCmdStamped cmd_msg;
  cmd_msg.header = msg->header;

  cmd_msg.header.stamp = ros::Time::now();  //Keep the time the command was given to the wheels_driver
  cmd_msg.vel_left = float(u_l);
  cmd_msg.vel_right = float(u_r);
  pub_wheelsCmd_.publish(cmd_msg);
}

