#include "../include/fusion_node.h"

namespace AWV_Fusion {

ESKF_Fusion::ESKF_Fusion(ros::NodeHandle nh) {
  double acc_n, gyr_n, acc_w, gyr_w, odom_n, gnss_n;
  std::string topic_imu, topic_gnss_base, topic_gnss_rover, topic_odom;
  double x, y, z;

  nh.param("acc_noise", acc_n, 1e-2);
  nh.param("gyr_noise", gyr_n, 1e-4);
  nh.param("acc_bias_noise", acc_w, 1e-6);
  nh.param("gyr_bias_noise", gyr_w, 1e-8);
  nh.param("odom_noise", odom_n, 1e-3);
  nh.param("gnss_noise", gnss_n, 1e-2);
  nh.param("p_I_GNSS_x", x, 0.);
  nh.param("p_I_GNSS_y", y, 0.);
  nh.param("p_I_GNSS_z", z, 0.);
  nh.param("topic_imu", topic_imu, std::string("filter/vectornav/imu"));
  nh.param("topic_gnss_base", topic_gnss_base,
           std::string("device/gnss_base/fix"));
  nh.param("topic_gnss_rover", topic_gnss_rover,
           std::string("device/gnss_rover/fix"));
  nh.param("topic_odom", topic_odom, std::string("/ctrl_base/odom"));

  init_pose_set_ = false;
  gnss_base_received_ = false;
  gnss_rover_received_ = false;
  init_heading_calculated_ = false;

  const Eigen::Vector3d p_I_GNSS(x, y, z);

  imu_sub_ = nh.subscribe(topic_imu, 10, &ESKF_Fusion::imu_callback, this);
  gnss_base_sub_ =
      nh.subscribe(topic_gnss_base, 10, &ESKF_Fusion::gnss_base_callback, this);
  gnss_rover_sub_ = nh.subscribe(topic_gnss_rover, 10,
                                 &ESKF_Fusion::gnss_rover_callback, this);
  odom_sub_ = nh.subscribe(topic_odom, 10, &ESKF_Fusion::odom_callback, this);

  path_pub_ = nh.advertise<nav_msgs::Path>("nav_path", 10);
  odom_pub_ = nh.advertise<nav_msgs::Odometry>("nav_odom", 10);

  open_file("gnss_data.txt", file_gnss_);
  open_file("state_data.txt", file_state_);
  open_file("odom_data.txt", file_odom_);

  eskf_ptr_ = std::make_shared<ESKF>(acc_n, gyr_n, acc_w, gyr_w, odom_n, gnss_n,
                                     p_I_GNSS);

  ROS_INFO("ESKF Fusion Start!");
}

ESKF_Fusion::~ESKF_Fusion() {
  file_gnss_.close();
  file_state_.close();
  file_odom_.close();
}

void ESKF_Fusion::imu_callback(const sensor_msgs::ImuConstPtr &imu_msg) {
  IMUPtr imu_data_ptr = std::make_shared<IMU>();
  imu_data_ptr->timestamp = imu_msg->header.stamp.toSec();
  imu_data_ptr->acc[0] = imu_msg->linear_acceleration.x;
  imu_data_ptr->acc[1] = imu_msg->linear_acceleration.y;
  imu_data_ptr->acc[2] = imu_msg->linear_acceleration.z;
  imu_data_ptr->gyro[0] = imu_msg->angular_velocity.x;
  imu_data_ptr->gyro[1] = imu_msg->angular_velocity.y;
  imu_data_ptr->gyro[2] = imu_msg->angular_velocity.z;
  if (!eskf_ptr_->process_IMU_Data(imu_data_ptr)) return;
  publish_save_state();
}

void ESKF_Fusion::gnss_base_callback(
    const sensor_msgs::NavSatFixConstPtr &gnss_msg) {
  if (gnss_msg->status.status == 0) {
    ROS_ERROR("Attention: Received Unstable GNSS data.");
    return;
  } else if (gnss_msg->status.status != 0 && !init_heading_calculated_) {
    gnss_base_data_ = *gnss_msg;
    last_gnss_base_time_ = gnss_msg->header.stamp.toSec();
    gnss_base_received_ = true;
    calculate_heading();
  }
  GNSSPtr gnss_data_ptr = std::make_shared<GNSS>();
  gnss_data_ptr->timestamp = gnss_msg->header.stamp.toSec();
  gnss_data_ptr->lla[0] = gnss_msg->latitude;
  gnss_data_ptr->lla[1] = gnss_msg->longitude;
  gnss_data_ptr->lla[2] = gnss_msg->altitude;
  gnss_data_ptr->cov =
      Eigen::Map<const Eigen::Matrix3d>(gnss_msg->position_covariance.data());
  if (!eskf_ptr_->process_GNSS_Data(gnss_data_ptr) && !init_heading_calculated_)
    return;
  file_gnss_ << std::fixed << std::setprecision(15) << gnss_data_ptr->timestamp
             << ", " << gnss_data_ptr->lla[0] << ", " << gnss_data_ptr->lla[1]
             << ", " << gnss_data_ptr->lla[2] << std::endl;
}

void ESKF_Fusion::gnss_rover_callback(
    const sensor_msgs::NavSatFixConstPtr &gnss_msg) {
  if (gnss_msg->status.status == 2 && !init_heading_calculated_) {
    gnss_rover_data_ = *gnss_msg;
    last_gnss_rover_time_ = gnss_msg->header.stamp.toSec();
    gnss_rover_received_ = true;
    calculate_heading();
  }
}

// void ESKF_Fusion::gnss_callback(
//     const sensor_msgs::NavSatFixConstPtr &gnss_msg_base,
//     const sensor_msgs::NavSatFixConstPtr &gnss_msg_rover) {
//   if (gnss_msg_base->status.status != 2 && gnss_msg_rover->status.status !=
//   2) {
//     ROS_ERROR("Attention: Received Unstable GNSS data.");
//     return;
//   } else if (gnss_msg_base->status.status == 2 && !init_heading_calculated_)
//   {
//     gnss_base_data_ = *gnss_msg_base;
//     last_gnss_base_time_ = gnss_msg_base->header.stamp.toSec();
//     gnss_base_received_ = true;
//     calculate_heading();
//   }
//   GNSSPtr gnss_data_ptr = std::make_shared<GNSS>();
//   gnss_data_ptr->timestamp = gnss_msg_base->header.stamp.toSec();
//   gnss_data_ptr->lla[0] = gnss_msg_base->latitude;
//   gnss_data_ptr->lla[1] = gnss_msg_base->longitude;
//   gnss_data_ptr->lla[2] = gnss_msg_base->altitude;
//   gnss_data_ptr->cov = Eigen::Map<const Eigen::Matrix3d>(
//       gnss_msg_base->position_covariance.data());
//   if (!eskf_ptr_->process_GNSS_Data(gnss_data_ptr) &&
//   !init_heading_calculated_)
//     return;
//   file_gnss_ << std::fixed << std::setprecision(15) <<
//   gnss_data_ptr->timestamp
//              << ", " << gnss_data_ptr->lla[0] << ", " <<
//              gnss_data_ptr->lla[1]
//              << ", " << gnss_data_ptr->lla[2] << std::endl;
// }

void ESKF_Fusion::calculate_heading() {
  if (init_heading_calculated_) return;  // Skip if already calculated
  ROS_INFO("Calculating initial GNSS heading...");
  if (gnss_base_received_ && gnss_rover_received_) {
    double time_diff = std::abs(last_gnss_base_time_ - last_gnss_rover_time_);
    if (time_diff < 0.1) {
      double lat1 = gnss_base_data_.latitude * M_PI / 180.0;
      double lon1 = gnss_base_data_.longitude * M_PI / 180.0;
      double lat2 = gnss_rover_data_.latitude * M_PI / 180.0;
      double lon2 = gnss_rover_data_.longitude * M_PI / 180.0;

      double dLon = lon2 - lon1;
      double y = sin(dLon) * cos(lat2);
      double x = cos(lat1) * sin(lat2) - sin(lat1) * cos(lat2) * cos(dLon);
      init_heading_ = atan2(y, x) * 180.0 / M_PI;
      init_heading_ = fmod(init_heading_ + 360.0, 360.0);

      ROS_INFO("Initial GNSS heading calculated: %.2f degrees", init_heading_);
      init_heading_calculated_ = true;

      gnss_base_received_ = false;
      gnss_rover_received_ = false;
    } else {
      ROS_ERROR(
          "GNSS base and rover timestamps are not synchronized within 0.1 "
          "seconds.");
    }
  }
}

void ESKF_Fusion::odom_callback(const nav_msgs::OdometryConstPtr &odom_msg) {
  ODOMPtr odom_data_ptr = std::make_shared<ODOM>();
  odom_data_ptr->timestamp = odom_msg->header.stamp.toSec();

  Eigen::Vector3d current_position(odom_msg->pose.pose.position.x,
                                   odom_msg->pose.pose.position.y,
                                   odom_msg->pose.pose.position.z);

  Eigen::Quaterniond current_orientation(
      odom_msg->pose.pose.orientation.w, odom_msg->pose.pose.orientation.x,
      odom_msg->pose.pose.orientation.y, odom_msg->pose.pose.orientation.z);

  if (!init_pose_set_) {
    init_position_ = current_position;
    init_orientation_ = current_orientation;
    init_pose_set_ = true;
    ROS_INFO("Initial position and orientation recorded.");
  }

  Eigen::Vector3d normalized_position = current_position - init_position_;

  Eigen::Quaterniond additional_rotation(Eigen::AngleAxisd(
      -init_heading_ * M_PI / 180.0, Eigen::Vector3d::UnitZ()));

  Eigen::Vector3d rotated_position =
      additional_rotation * init_orientation_.inverse() * normalized_position;

  odom_data_ptr->pose[0] = rotated_position.x();
  odom_data_ptr->pose[1] = rotated_position.y();
  odom_data_ptr->pose[2] = rotated_position.z();

  odom_data_ptr->orientation.x() = current_orientation.x();
  odom_data_ptr->orientation.y() = current_orientation.y();
  odom_data_ptr->orientation.z() = current_orientation.z();
  odom_data_ptr->orientation.w() = current_orientation.w();

  odom_data_ptr->linear_vel[0] = odom_msg->twist.twist.linear.x;
  odom_data_ptr->linear_vel[1] = odom_msg->twist.twist.linear.y;
  odom_data_ptr->linear_vel[2] = odom_msg->twist.twist.linear.z;
  odom_data_ptr->ang_vel[0] = odom_msg->twist.twist.angular.x;
  odom_data_ptr->ang_vel[1] = odom_msg->twist.twist.angular.y;
  odom_data_ptr->ang_vel[2] = odom_msg->twist.twist.angular.z;
  odom_data_ptr->cov =
      Eigen::Map<const Eigen::Matrix3d>(odom_msg->pose.covariance.data());

  if (!eskf_ptr_->process_ODOM_Data(odom_data_ptr)) return;

  file_odom_ << std::fixed << std::setprecision(15) << odom_data_ptr->pose[0]
             << ", " << odom_data_ptr->pose[1] << ", " << odom_data_ptr->pose[2]
             << std::endl;
}

void ESKF_Fusion::publish_save_state(void) {
  // publish the odometry
  std::string fixed_id = "global";
  nav_msgs::Odometry odom_msg;
  odom_msg.header.frame_id = fixed_id;
  odom_msg.header.stamp = ros::Time::now();
  Eigen::Isometry3d T_wb = Eigen::Isometry3d::Identity();
  T_wb.linear() = eskf_ptr_->state_ptr_->R_W_I.matrix();
  T_wb.translation() = eskf_ptr_->state_ptr_->P_W_I;
  tf::poseEigenToMsg(T_wb, odom_msg.pose.pose);
  tf::vectorEigenToMsg(eskf_ptr_->state_ptr_->V_W_I,
                       odom_msg.twist.twist.linear);
  Eigen::Matrix3d P_pp =
      eskf_ptr_->state_ptr_->cov.block<3, 3>(0, 0);  // position covariance
  Eigen::Matrix3d P_po = eskf_ptr_->state_ptr_->cov.block<3, 3>(
      0, 6);  // position rotation covariance
  Eigen::Matrix3d P_op = eskf_ptr_->state_ptr_->cov.block<3, 3>(
      6, 0);  // rotation position covariance
  Eigen::Matrix3d P_oo =
      eskf_ptr_->state_ptr_->cov.block<3, 3>(6, 6);  // rotation covariance
  Eigen::Matrix<double, 6, 6, Eigen::RowMajor> P_imu_pose =
      Eigen::Matrix<double, 6, 6>::Zero();
  P_imu_pose << P_pp, P_po, P_op, P_oo;
  for (int i = 0; i < 36; i++)
    odom_msg.pose.covariance[i] = P_imu_pose.data()[i];
  odom_pub_.publish(odom_msg);

  // publish the path
  geometry_msgs::PoseStamped pose_stamped;
  pose_stamped.header = odom_msg.header;
  pose_stamped.pose = odom_msg.pose.pose;
  nav_path_.header = pose_stamped.header;
  nav_path_.poses.push_back(pose_stamped);
  path_pub_.publish(nav_path_);

  // save state p q lla
  Eigen::Vector3d lla;
  convert_enu_to_lla(eskf_ptr_->init_lla_, eskf_ptr_->state_ptr_->P_W_I, &lla);
  Eigen::Quaterniond q_G_I(eskf_ptr_->state_ptr_->R_W_I.matrix());
  file_state_ << std::fixed << std::setprecision(15)
              << eskf_ptr_->state_ptr_->timestamp << ", "
              << eskf_ptr_->state_ptr_->P_W_I[0] << ", "
              << eskf_ptr_->state_ptr_->P_W_I[1] << ", "
              << eskf_ptr_->state_ptr_->P_W_I[2] << ", " << q_G_I.x() << ", "
              << q_G_I.y() << ", " << q_G_I.z() << ", " << q_G_I.w() << ", "
              << lla[0] << ", " << lla[1] << ", " << lla[2] << std::endl;
}

bool ESKF_Fusion::file_exist(const std::string &filename) {
  std::ifstream file(filename);
  return file.good();
}

void ESKF_Fusion::open_file(const std::string &filename, std::ofstream &file) {
  std::filesystem::path currentPath = std::filesystem::current_path();
  std::filesystem::path file_path = currentPath / filename;
  if (!file_exist(filename)) {
    std::ofstream file(file_path);
    if (file) {
      ROS_INFO("File created at: %s", file_path.c_str());
    } else {
      ROS_ERROR("Failed to create file at: %s", file_path.c_str());
    }
  } else {
    ROS_INFO("File already exists at: %s", file_path.c_str());
  }
  file.open(filename, std::ios::out | std::ios::trunc);
}

}  // namespace AWV_Fusion