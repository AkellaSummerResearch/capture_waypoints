
#include "ros/ros.h"
#include <geometry_msgs/PoseStamped.h>
#include <mutex>
#include <thread>
#include <iostream>
#include <fstream>

#include <capture_waypoints/helper.h>

class CaptureWaypoints {
  ros::NodeHandle nh_;
  std::string in_pose_topic_, out_filename_;
  bool pose_updated_;  // Flag indicating that the first pose message has arrived
  ros::Subscriber pose_sub_;
  std::thread h_capture_thread_;
  geometry_msgs::PoseStamped current_pose_;
  std::mutex current_pose_mutex_;
  std::vector<geometry_msgs::PoseStamped> pose_list_;

 public:
  CaptureWaypoints(ros::NodeHandle *nh) {
    nh_= *nh;
    nh_.getParam("input_pose_topic", in_pose_topic_);
    nh_.getParam("output_file", out_filename_);

    // Subscribe to pose topic for capturing waypoints
    pose_sub_ = nh->subscribe(in_pose_topic_, 10, &CaptureWaypoints::PoseCallback, this);
    pose_updated_ = false;

    // Create thread that waits for keyboard commands to capture waypoints
    h_capture_thread_ = std::thread(&CaptureWaypoints::CaptureWpsTask, this);

  }

  void CaptureWpsTask() {
  	std::string input_str;
  	geometry_msgs::PoseStamped current_pose;

  	std::cout << "Press enter to capture a waypoint. Type ''stop'' to save file and end program." << std::endl;
  	while(true) {
  		std::getline(std::cin, input_str);
  		if(input_str.compare("stop") == 0) {
  			std::cout << "Program is ending! Saving to file " << out_filename_ << std::endl;
  			this->SaveWaypoints();
  			break;
  		}

	  	current_pose_mutex_.lock();
	  	current_pose = current_pose_;
	  	current_pose_mutex_.unlock();

	  	pose_list_.push_back(current_pose);

	  	Eigen::Vector3d rpy = helper::quat2rpy(current_pose.pose.orientation);
	  	ROS_INFO("%zd) X: %4.2f  Y: %4.2f  Z: %4.2f  Yaw: %4.2f", pose_list_.size(), current_pose.pose.position.x, 
	  		       current_pose.pose.position.y, current_pose.pose.position.z, helper::rad2deg(rpy[2]));
  	}
  }

  void SaveWaypoints() {
  	std::ofstream out_file;
  	out_file.open(out_filename_);
  	if(!out_file.is_open()) {
  		ROS_WARN("Could not open file for saving waypoints!");
  		return;
  	}
  	for(uint i = 0; i < pose_list_.size(); i++) {
  		Eigen::Vector3d rpy = helper::quat2rpy(pose_list_[i].pose.orientation);
  		out_file << pose_list_[i].pose.position.x << " " << pose_list_[i].pose.position.y << " "
  				 << pose_list_[i].pose.position.z << " " << rpy[2] << std::endl;
  	}
  	out_file.close();
  }

  void PoseCallback(const geometry_msgs::PoseStamped::ConstPtr &msg) {
  	pose_updated_ = true;

  	current_pose_mutex_.lock();
  	current_pose_ = *msg;
  	current_pose_mutex_.unlock();
  }

};

int main(int argc, char **argv) {
  ros::init(argc, argv, "waypoint_capture_node");
  ros::NodeHandle node("~");

  CaptureWaypoints obj_capture_wps(&node);

  ros::spin();

  return 0;
}