#include "ros/ros.h" // main ROS include
#include "std_msgs/Float32.h" // number message datatype
#include <ros/console.h>
#include <yaml-cpp/yaml.h>
#include <boost/filesystem.hpp>
#include "geometry_msgs/Point.h"
#include "duckietown_msgs/WheelsCmd.h"
#include "duckietown_msgs/WheelsCmdStamped.h"
#include "geometry_msgs/Pose2D.h"
#include "duckietown_msgs/Pose2DStamped.h"
#include "duckietown_msgs/Twist2DStamped.h"
#include "duckietown_msgs/Vector2D.h"
#include <visualization_msgs/Marker.h>
#include <std_srvs/Empty.h>
#include <cmath> // needed for nan
#include <stdint.h>
#include <fstream>

class forward_kinematics_node
{
public:
forward_kinematics_node(); // constructor

// class variables_
private:
  ros::NodeHandle nh_; // interface to this node
  ros::Subscriber sub_dutyToWheelOmega_;
  ros::Subscriber sub_wheelOmegaToTwist_;
  ros::Subscriber sub_twistToOdometry_;

  ros::Publisher pub_dutyToWheelOmega_;
  ros::Publisher pub_wheelOmegaToTwist_; 
  ros::Publisher pub_twistToOdometry_; 
  ros::Publisher pub_odomTrajectory_; 
 
  // TODO: these variables should be computed/given by calibration
  double K_l_; // duty to left wheel rotation rate constant
  double K_r_; // duty to right wheel rotation rate constant

  double radius_l_; // radius of the left wheel
  double radius_r_; // radius of the right wheel
  double baseline_lr_; //distance between the center of the two wheels

  double previousTimestamp_;
  geometry_msgs::Pose2D odomPose_;

  visualization_msgs::Marker odomTrajectory_;
  int odomSubsampleStep_; // we subsample odometric trajectory to visualize it in rviz
  int odomSubsampleCount_;

	// callback function declarations
  void dutyToWheelOmegaCallback(duckietown_msgs::WheelsCmdStamped::ConstPtr const& msg);
  void wheelOmegaToTwistCallback(duckietown_msgs::WheelsCmdStamped::ConstPtr const& msg);
	void twistToOdometryCallback(duckietown_msgs::Twist2DStamped::ConstPtr const& msg);
};

// program entry point
int main(int argc, char *argv[])
{
	// initialize the ROS client API, giving the default node name
	ros::init(argc, argv, "forward_kinematics_node");
	forward_kinematics_node node;
	ros::spin(); // enter the ROS main loop
	return 0;
}

// class constructor; subscribe to topics and advertise intent to publish
forward_kinematics_node::forward_kinematics_node() :
K_l_(0.1), K_r_(0.1), radius_l_(0.02), radius_r_(0.02), baseline_lr_(0.1) {

	// subscribe to the topics
  sub_dutyToWheelOmega_ = nh_.subscribe("/starducks/wheels_driver_node/wheels_cmd", 1, &forward_kinematics_node::dutyToWheelOmegaCallback, this);
  pub_dutyToWheelOmega_ = nh_.advertise<duckietown_msgs::WheelsCmdStamped>("/starducks/wheels_driver_node/wheelsOmega", 1);

  sub_wheelOmegaToTwist_ = nh_.subscribe("/starducks/wheels_driver_node/wheelsOmega", 1, &forward_kinematics_node::wheelOmegaToTwistCallback, this);
  pub_wheelOmegaToTwist_ = nh_.advertise<duckietown_msgs::Twist2DStamped>("/starducks/wheels_driver_node/twist", 1); 

  sub_twistToOdometry_ = nh_.subscribe("/starducks/wheels_driver_node/twist", 1, &forward_kinematics_node::twistToOdometryCallback, this); 
  pub_twistToOdometry_ = nh_.advertise<duckietown_msgs::Pose2DStamped>("/starducks/wheels_driver_node/odometricPose", 1); 

  // the following is only to visualize the odometric trajectory
  pub_odomTrajectory_ = nh_.advertise<visualization_msgs::Marker>("/starducks/odometricTrajectory", 1); 
  odomTrajectory_.header.frame_id = "/odom";
  odomTrajectory_.ns = "odometricTrajectory";
  odomTrajectory_.action = visualization_msgs::Marker::ADD;
  odomTrajectory_.pose.orientation.w = 1.0;
  odomTrajectory_.id = 1;
  odomTrajectory_.type = visualization_msgs::Marker::LINE_STRIP;
  odomTrajectory_.scale.x = 0.1;
  odomTrajectory_.color.r = 1.0;
  odomTrajectory_.color.a = 1.0;
  odomSubsampleCount_ = odomSubsampleStep_;
}

///////////////////////////////////////////////////////////////////////////////////////////
void forward_kinematics_node::dutyToWheelOmegaCallback(duckietown_msgs::WheelsCmdStamped::ConstPtr const& msg){

  // Convertion from motor duty to motor rotation rate (currently a naive multiplication)
  double omega_r =  K_r_ * msg->vel_right;
  double omega_l =  K_l_ * msg->vel_left;

  // put in a message and publish
  duckietown_msgs::WheelsCmdStamped wheelOmega_rl_msg;
  wheelOmega_rl_msg.header = msg->header;
  wheelOmega_rl_msg.vel_right = omega_r;
  wheelOmega_rl_msg.vel_left = omega_l;
  pub_dutyToWheelOmega_.publish(wheelOmega_rl_msg);   
}

///////////////////////////////////////////////////////////////////////////////////////////
void forward_kinematics_node::wheelOmegaToTwistCallback(duckietown_msgs::WheelsCmdStamped::ConstPtr const& msg){

  // get wheels rotation rates from message
  double omega_r =  msg->vel_right;
  double omega_l =  msg->vel_left;   

  // compute linear and angular velocity of the platform
  double v = (radius_r_ * omega_r + radius_l_* omega_l) / 2;
  double omega =  (radius_r_ * omega_r - radius_l_* omega_l) / baseline_lr_;

  // put in a message and publish
  duckietown_msgs::Twist2DStamped twist_msg;
  twist_msg.header = msg->header;
  twist_msg.v = v;
  twist_msg.omega = omega;
  pub_wheelOmegaToTwist_.publish(twist_msg);  
}

///////////////////////////////////////////////////////////////////////////////////////////
void forward_kinematics_node::twistToOdometryCallback(duckietown_msgs::Twist2DStamped::ConstPtr const& msg){

  if(msg->header.seq <= 1){
   // first odometry message, we only record time stamp
  }else{    
    double v = msg->v;
    double omega = msg->omega; 
    double deltaT = msg->header.stamp.sec - previousTimestamp_;   

    // odomPose_
    double theta_tm1 = odomPose_.theta; // orientation at time t
    double theta_t = theta_tm1 + omega * deltaT; // orientation at time t+1
    
    if (fabs(omega) <= 0.0001){
      // straight line
      odomPose_.x = odomPose_.x + sin(theta_tm1) * v;
      odomPose_.y = odomPose_.y + cos(theta_tm1) * v;
    }else{
      // arc of circle, see "Probabilitic robotics"
      double v_w_ratio = v / omega;
      odomPose_.x = odomPose_.x - v_w_ratio * sin(theta_tm1) + v_w_ratio * sin(theta_t);
      odomPose_.y = odomPose_.y + v_w_ratio * cos(theta_tm1) - v_w_ratio * sin(theta_t);
    }
    odomPose_.theta = theta_t;

    // put in msg and publish
    duckietown_msgs::Pose2DStamped odomPose_msg;  
    odomPose_msg.header = msg->header; 
    odomPose_msg.x = odomPose_.x; 
    odomPose_msg.y = odomPose_.y; 
    odomPose_msg.theta = odomPose_.theta; 
    pub_twistToOdometry_.publish(odomPose_msg);   

    odomSubsampleCount_ -= 1;
    if(odomSubsampleCount_ <= 0){
      geometry_msgs::Point p;
      p.x = odomPose_.x;
      p.y = odomPose_.y;
      p.z = 0.0;
      odomTrajectory_.points.push_back(p);
      pub_odomTrajectory_.publish(odomTrajectory_);
      odomSubsampleCount_ = odomSubsampleStep_; // reset counter
    }
  } 
  previousTimestamp_ = msg->header.stamp.sec; // update time for next integration
}
