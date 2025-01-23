#ifndef AWV_FUSION_ESKF_H
#define AWV_FUSION_ESKF_H

#include <ros/ros.h>
#include <vector>
#include "../include/utils.h"

namespace AWV_Fusion {

class ESKF {
 public:
  ESKF(double acc_n, double gyr_n, double acc_w, double gyr_w, double odom_n,
       double gnss_n, Eigen::Vector3d p_I_GNSS);
  StatePtr state_ptr_;
  Eigen::Vector3d p_I_GNSS_;
  Eigen::Vector3d init_lla_;
  Eigen::VectorXd delta_x_;
  std::deque<IMUPtr> imu_buffer_;
  IMUPtr last_imu_ptr_;

  bool process_IMU_Data(IMUPtr imu_data_ptr);
  bool process_GNSS_Data(GNSSPtr gnss_data_ptr);
  bool process_ODOM_Data(ODOMPtr odom_data_ptr);
  bool initialize();
  void predict(IMUPtr last_imu_ptr, IMUPtr cur_imu_ptr);
  void update_gnss(GNSSPtr gnss_data_ptr);
  void update_odom(ODOMPtr odom_data_ptr);
  void state_update();

 private:
  double acc_noise_;
  double gyro_noise_;
  double acc_bias_noise_;
  double gyro_bias_noise_;
  double odom_noise_;
  double gnss_noise_;

  bool imu_initialized_;
  bool gnss_initialized_;
  bool first_gnss_;
  bool second_gnss_;
};

using ESKFPtr = std::shared_ptr<ESKF>;

}  // namespace AWV_Fusion

#endif  // AWV_FUSION_ESKF_H
