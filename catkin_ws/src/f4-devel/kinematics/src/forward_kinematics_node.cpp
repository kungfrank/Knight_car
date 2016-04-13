#include "ros/ros.h" // main ROS include
#include <ros/console.h>
#include "duckietown_msgs/WheelsCmdStamped.h"
#include "duckietown_msgs/Pose2DStamped.h"
#include "duckietown_msgs/Twist2DStamped.h"
#include <string>
using namespace std;

class forward_kinematics_node
{
public:
forward_kinematics_node(); // constructor

// class variables_
private:
  ros::NodeHandle nh_; // interface to this node
  string node_name_;
  string veh_name_;

  ros::Subscriber sub_wheelsCmd_;

  ros::Publisher pub_vehicleVelocity_;
  ros::Publisher pub_vehiclePose_;

  double K_l_; // duty to left wheel rotation rate constant
  double K_r_; // duty to right wheel rotation rate constant
  double radius_l_; // radius of the left wheel
  double radius_r_; // radius of the right wheel
  double baseline_lr_; //distance between the center of the two wheels

  ros::Time previousTimestamp_;
  duckietown_msgs::Pose2DStamped odomPose_;

  // callback function declarations
  void wheelsCmdCallback(duckietown_msgs::WheelsCmdStampedConstPtr const& msg);
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
forward_kinematics_node::forward_kinematics_node() : nh_("~"), node_name_("forward_kinematics_node"),
                                                     previousTimestamp_(0)
{

  //Get parameters
  nh_.param("K_l", K_l_, 25.0);
  nh_.param("K_r", K_r_, 25.0);
  nh_.param("radius_l", radius_l_, 0.02);
  nh_.param("radius_r", radius_r_, 0.02);
  nh_.param("baseline_lr", baseline_lr_, 0.1);

  //Get the vehicle name
  veh_name_ = ros::this_node::getNamespace();
  veh_name_.erase(std::remove(veh_name_.begin(),veh_name_.end(), '/'), veh_name_.end());
  ROS_INFO_STREAM("[" << node_name_<< "] Vehicle Name: " << veh_name_);

  // subscribe to the topics
  sub_wheelsCmd_ = nh_.subscribe("wheels_cmd", 1, &forward_kinematics_node::wheelsCmdCallback, this);

  // setup the publishers
  pub_vehicleVelocity_ = nh_.advertise<duckietown_msgs::Twist2DStamped>("velocity", 1);
  pub_vehiclePose_ = nh_.advertise<duckietown_msgs::Pose2DStamped>("pose", 1);

  ROS_INFO_STREAM("[" << node_name_ << "] has started.");
}

///////////////////////////////////////////////////////////////////////////////////////////
void forward_kinematics_node::wheelsCmdCallback(duckietown_msgs::WheelsCmdStampedConstPtr const& msg)
{
  // Convertion from motor duty to motor rotation rate (currently a naive multiplication)
  double omega_r =  K_r_ * msg->vel_right;
  double omega_l =  K_l_ * msg->vel_left;


  // Compute linear and angular velocity of the platform
  double v = (radius_r_ * omega_r + radius_l_* omega_l) / 2.0;
  double omega =  (radius_r_ * omega_r - radius_l_* omega_l) / baseline_lr_;

  // stuff the velocities in a message and publish
  duckietown_msgs::Twist2DStamped twist_msg;
  twist_msg.header = msg->header;
  twist_msg.v = (float)v;
  twist_msg.omega = (float)omega;
  pub_vehicleVelocity_.publish(twist_msg);


  // Compute the final pose by integration
  // We should skip the first odometry message because we won't know the delta t
  if(previousTimestamp_.toSec() > 0)
  {
    double deltaT = (msg->header.stamp - previousTimestamp_).toSec();
    double theta_tm1 = odomPose_.theta; // orientation at time t
    double theta_t = theta_tm1 + omega * deltaT; // orientation at time t+1

    if (fabs(omega) <= 0.0001){
      // straight line
      odomPose_.x += cos(theta_tm1) * v * deltaT;
      odomPose_.y += sin(theta_tm1) * v * deltaT;
    }else{
      // arc of circle, see "Probabilitic robotics"
      double v_w_ratio = v / omega;
      odomPose_.x += v_w_ratio * sin(theta_t) - v_w_ratio * sin(theta_tm1);
      odomPose_.y += v_w_ratio * cos(theta_tm1) - v_w_ratio * cos(theta_t);
    }
    odomPose_.theta = theta_t;

    odomPose_.header = msg->header;
    odomPose_.header.frame_id = veh_name_;

    //Publish the pose message
    pub_vehiclePose_.publish(odomPose_);
  }
  previousTimestamp_ = msg->header.stamp; // update time for next integration
}