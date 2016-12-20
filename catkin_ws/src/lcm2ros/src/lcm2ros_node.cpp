#include <cstdlib>
#include <string>
#include <ros/ros.h>
#include <lcm/lcm-cpp.hpp>

#include "lcmtypes/bot_core/pose_t.hpp"

#include <nav_msgs/Odometry.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Quaternion.h>
#include <std_msgs/Float64.h>
#include <map>

#include <duckietown_msgs/Twist2DStamped.h>
#include <semantic_mapping_msgs/StreetNaviTurn.h>
using namespace std;

class LCM2ROS{
	public:
		LCM2ROS(boost::shared_ptr<lcm::LCM> &lcm_, ros::NodeHandle &nh_, std::string &veh_);
		~LCM2ROS() {}
	private:
		boost::shared_ptr<lcm::LCM> lcm_;
		ros::NodeHandle nh_;
        std::string veh_; 

		void poseBodyHandler(const lcm::ReceiveBuffer* rbuf, const std::string &channel, const bot_core::pose_t* msg);
		//ros::Publisher pose_body_pub_;
		ros::Publisher quaternion_pub_;
		ros::Publisher point_pub_;
		ros::NodeHandle* rosnode;
};

LCM2ROS::LCM2ROS(boost::shared_ptr<lcm::LCM> &lcm_, ros::NodeHandle &nh_, std::string &veh_ ): lcm_(lcm_),nh_(nh_),veh_(veh_) {
	std::stringstream lcm_pose_topic;
	lcm_pose_topic << "POSE_T_" << veh_;
	lcm_->subscribe(lcm_pose_topic.str(), &LCM2ROS::poseBodyHandler, this);
	//pose_body_pub_ = nh_.advertise<nav_msgs::Odometry>("/pose_body",10);
	std::stringstream topic;
	std::stringstream topic1;
	topic << "/" << veh_ << "/lcm2ros_node/mocap/position";
    topic1 << "/" << veh_ << "/lcm2ros_node/mocap/orientation";
	point_pub_ = nh_.advertise<geometry_msgs::Point>(topic.str(),10);
	quaternion_pub_ = nh_.advertise<geometry_msgs::Quaternion>(topic1.str(),10);
	rosnode = new ros::NodeHandle();
}

void LCM2ROS::poseBodyHandler(const lcm::ReceiveBuffer* rbuf, const std::string &channel, const bot_core::pose_t* msg){
	//nav_msgs::Odometry msgout;
	geometry_msgs::Point pose_position;
	geometry_msgs::Quaternion pose_orientation;
	//msgout.header.stamp = ros::Time().fromSec(msg->utime*1E-6);
	//pose_position= ros::Time().fromSec(msg->utime*1E-6);
    pose_position.x = msg->pos[0];
	pose_position.y = msg->pos[1];
	pose_position.z = msg->pos[2];
	pose_orientation.x = msg->orientation[0];
	pose_orientation.y = msg->orientation[1];
	pose_orientation.z = msg->orientation[2];
	pose_orientation.w = msg->orientation[3];
	//pose_body_pub_.publish(msgout);
	cout << "get mocap_pose_t position x: " << msg->pos[0] << endl;
	cout << "get mocap_pose_t position y: " << msg->pos[1] << endl;
	cout << "get mocap_pose_t position z: " << msg->pos[2] << endl;
	cout << "get mocap_pose_t orientation x: " << msg->orientation[0] << endl;
	cout << "get mocap_pose_t orientation y: " << msg->orientation[1] << endl;
	cout << "get mocap_pose_t orientation z: " << msg->orientation[2] << endl;
	cout << "get mocap_pose_t orientation w: " << msg->orientation[3] << endl;
	point_pub_.publish(pose_position);
	quaternion_pub_.publish(pose_orientation);
	//if(msg->pos[0] == 1){
	//	turncmd.turn = "right";
	//	turn_pub_.publish(turncmd);
    //}else if(msg->pos[0] == -1){
    //    turncmd.turn = "left";
    //    turn_pub_.publish(turncmd);
	//}else{
	//	turncmd.turn = "straight";
	//	turn_pub_.publish(turncmd);
	//}
}

int main(int argc, char** argv){
	ros::init(argc, argv, "lcm2ros", ros::init_options::NoSigintHandler);
	boost::shared_ptr<lcm::LCM> lcm(new lcm::LCM);
	if(!lcm->good()){
		std::cerr << "ERROR: lcm is not goood()" << std::endl;
	}	
	ros::NodeHandle nh("~");
    std::string veh;

	//nh.param<std::string>("lcm2ros_node/veh", veh, "trabant");
	nh.getParam("/lcm2ros_node/veh", veh);
    cout << "veh = " << veh <<endl;

	LCM2ROS handler_turn(lcm,nh,veh);
	cout << "lcm2ros translator ready";
    //ROS_ERROR("LCM@ROS Tranlator Ready");

	while(0 == lcm->handle());
	return 0;
}
