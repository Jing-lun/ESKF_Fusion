#ifndef AWV_FUSION_ESKF_FUSION_H
#define AWV_FUSION_ESKF_FUSION_H

#include <eigen_conversions/eigen_msg.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <ros/package.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <string>
#include "../include/eskf.h"

namespace AWV_Fusion {

class ESKF_Fusion {
 public:
  ESKF_Fusion(ros::NodeHandle nh);
  ~ESKF_Fusion();

  void imu_callback(const sensor_msgs::ImuConstPtr &imu_msg);
  void gnss_base_callback(const sensor_msgs::NavSatFixConstPtr &gnss_msg);
  void gnss_rover_callback(const sensor_msgs::NavSatFixConstPtr &gnss_msg);
  void odom_callback(const nav_msgs::OdometryConstPtr &odom_msg);

  void calculate_heading();
  void publish_save_state();
  bool file_exist(const std::string &name);
  void open_file(const std::string &name, std::ofstream &file);

 private:
  ros::Subscriber imu_sub_;
  ros::Subscriber gnss_base_sub_;
  ros::Subscriber gnss_rover_sub_;
  ros::Subscriber odom_sub_;
  ros::Publisher path_pub_;
  ros::Publisher odom_pub_;
  nav_msgs::Path nav_path_;
  sensor_msgs::NavSatFix gnss_base_data_;
  sensor_msgs::NavSatFix gnss_rover_data_;

  ESKFPtr eskf_ptr_;

  bool init_pose_set_;
  Eigen::Vector3d init_position_;
  Eigen::Quaterniond init_orientation_;

  bool gnss_base_received_;
  bool gnss_rover_received_;
  double last_gnss_base_time_;
  double last_gnss_rover_time_;
  bool init_heading_calculated_;
  double init_heading_;

  // log files
  std::ofstream file_gnss_;
  std::ofstream file_state_;
  std::ofstream file_odom_;
};

}  // namespace AWV_Fusion

#endif  // AWV_FUSION_ESKF_FUSION_H