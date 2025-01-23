#include "../include/eskf.h"

namespace AWV_Fusion {

ESKF::ESKF(double acc_n, double gyro_n, double acc_w, double gyro_w,
           double odom_n, double gnss_n, Eigen::Vector3d p_I_GNSS) {
  acc_noise_ = acc_n;
  gyro_noise_ = gyro_n;
  acc_bias_noise_ = acc_w;
  gyro_bias_noise_ = gyro_w;
  odom_noise_ = odom_n;
  gnss_noise_ = gnss_n;
  p_I_GNSS_ = p_I_GNSS;
  state_ptr_ = std::make_shared<State>();
  delta_x_ = Eigen::VectorXd::Zero(18);
  imu_initialized_ = false;
  gnss_initialized_ = false;
}
bool ESKF::process_IMU_Data(IMUPtr imu_data_ptr) {
  if (!imu_initialized_) {
    imu_buffer_.push_back(imu_data_ptr);
    if (imu_buffer_.size() > IMU_Buffer_Size) imu_buffer_.pop_front();
    return false;
  }
  predict(last_imu_ptr_, imu_data_ptr);
  last_imu_ptr_ = imu_data_ptr;
  return true;
}

void ESKF::predict(IMUPtr last_imu_ptr, IMUPtr cur_imu_ptr) {
  double dt = cur_imu_ptr->timestamp - last_imu_ptr->timestamp;
  double dt_2 = dt * dt;

  State last_state = *state_ptr_;

  // timestamp
  state_ptr_->timestamp = cur_imu_ptr->timestamp;

  // P V R
  Eigen::Vector3d acc_unbias = cur_imu_ptr->acc - last_state.acc_bias;
  Eigen::Vector3d gyr_unbias = cur_imu_ptr->gyro - last_state.gyro_bias;
  Eigen::Vector3d acc_nominal = last_state.R_W_I * acc_unbias + state_ptr_->g_W;
  state_ptr_->P_W_I =
      last_state.P_W_I + last_state.V_W_I * dt + 0.5 * acc_nominal * dt_2;
  state_ptr_->V_W_I = last_state.V_W_I + acc_nominal * dt;
  state_ptr_->R_W_I = last_state.R_W_I * SO3d::exp(gyr_unbias * dt);

  // Fx = 18 * 18
  //[I    I*dt    0    0    0     0]
  //[0    I -R[a]^dt -Rdt   0  I*dt]
  //[0    0  Rt{w*dt}  0  Idt     0]
  //[0    0       0    I    0     0]
  //[0    0       0    0    I     0]
  //[0    0       0    0    0     I]

  Eigen::Matrix<double, 18, 18> Fx = Eigen::Matrix<double, 18, 18>::Identity();
  Fx.block<3, 3>(0, 3) = Eigen::Matrix3d::Identity() * dt;
  Fx.block<3, 3>(3, 6) =
      -state_ptr_->R_W_I.matrix() * SO3d::hat(acc_unbias) * dt;
  Fx.block<3, 3>(3, 9) = -state_ptr_->R_W_I.matrix() * dt;
  Fx.block<3, 3>(3, 15) = Eigen::Matrix3d::Identity() * dt;
  Fx.block<3, 3>(6, 6) = SO3d::exp(-gyr_unbias * dt).matrix();
  Fx.block<3, 3>(6, 12) = -Eigen::Matrix3d::Identity() * dt;

  // Qi = 18 * 18
  //[0      0          0        0        0        0]
  //[0  dt_2*acc_n     0        0        0        0]
  //[0      0     dt_2*gyr_n    0        0        0]
  //[0      0          0     dt*acc_w    0        0]
  //[0      0          0        0     dt*gyo_w    0]
  //[0      0          0        0        0        0]

  Eigen::Matrix<double, 18, 18> Qi = Eigen::Matrix<double, 18, 18>::Zero();
  Qi.block<12, 12>(3, 3) = Eigen::Matrix<double, 12, 12>::Identity();
  Qi.block<3, 3>(3, 3) *= dt_2 * acc_noise_;
  Qi.block<3, 3>(6, 6) *= dt_2 * gyro_noise_;
  Qi.block<3, 3>(9, 9) *= dt * acc_bias_noise_;
  Qi.block<3, 3>(12, 12) *= dt * gyro_bias_noise_;

  //   P = Fx * P * Fxt + Qi
  //     -> a simple version of P = Fx * P * Fxt + Fi * Qi * Fit
  state_ptr_->cov = Fx * last_state.cov * Fx.transpose() + Qi;
}

void ESKF::update_gnss(GNSSPtr gnss_data_ptr) {
  Eigen::Vector3d p_W_GNSS;
  convert_lla_to_enu(init_lla_, gnss_data_ptr->lla, &p_W_GNSS);

  Eigen::Vector3d &P_W_I = state_ptr_->P_W_I;
  // Eigen::Vector3d residual = p_W_GNSS - (P_W_I + state_ptr_->R_W_I *
  // p_I_GNSS_);
  Eigen::Vector3d residual = p_W_GNSS - P_W_I;
  Eigen::Matrix<double, 3, 18> H;
  H.setZero();
  H.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity();
  // H.block<3, 3>(0, 6) = -state_ptr_->R_W_I.matrix() * SO3d::hat(p_I_GNSS_);
  Eigen::Matrix3d V = gnss_noise_ * Eigen::Matrix3d::Identity();

  // K = P * Ht * ( H * P * Ht + V).inverse();
  // dx = K * (y - h(x))
  // P = (I - KH) * P * (I - KH)t + K * V * Kt
  //   ->  original version has a poor numerical stability, as its outcome is
  //   not guaranteed to be symmetric nor positive definite, thus make it to be
  //   a symmetric and positive Joseph form
  const Eigen::MatrixXd &P = state_ptr_->cov;
  const Eigen::MatrixXd K =
      P * H.transpose() * (H * P * H.transpose() + V).inverse();
  delta_x_ = K * residual;

  const Eigen::MatrixXd I_KH =
      Eigen::Matrix<double, 18, 18>::Identity() - K * H;
  // state_ptr_->cov = I_KH * state_ptr_->cov;
  state_ptr_->cov = I_KH * P * I_KH.transpose() + K * V * K.transpose();
  state_update();
}

bool ESKF::process_GNSS_Data(GNSSPtr gnss_data_ptr) {
  if (!imu_initialized_) {
    if (imu_buffer_.size() < IMU_Buffer_Size) {
      ROS_WARN("Wait. Insufficient IMU data.");
      return false;
    }
    last_imu_ptr_ = imu_buffer_.back();
    if (std::abs(last_imu_ptr_->timestamp - gnss_data_ptr->timestamp) > 0.15) {
      ROS_ERROR("GNSS and IMU are not sychonized.");
      return false;
    }
    if (!initialize()) return false;
    init_lla_ = gnss_data_ptr->lla;
    imu_initialized_ = true;
    gnss_initialized_ = true;
  }

  update_gnss(gnss_data_ptr);
  return true;
}

void ESKF::update_odom(ODOMPtr odom_data_ptr) {
  Eigen::Vector3d &P_W_I = state_ptr_->P_W_I;
  Eigen::Vector3d residual = odom_data_ptr->pose - P_W_I;
  Eigen::Matrix<double, 3, 18> H;
  H.setZero();
  H.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity();
  Eigen::Matrix3d V = odom_noise_ * Eigen::Matrix3d::Identity();

  // K = P * Ht * ( H * P * Ht + V).inverse();
  // dx = K * (y - h(x))
  // P = (I - KH) * P * (I - KH)t + K * V * Kt
  const Eigen::MatrixXd &P = state_ptr_->cov;
  const Eigen::MatrixXd K =
      P * H.transpose() * (H * P * H.transpose() + V).inverse();
  delta_x_ = K * residual;

  const Eigen::MatrixXd I_KH =
      Eigen::Matrix<double, 18, 18>::Identity() - K * H;
  // state_ptr_->cov = I_KH * state_ptr_->cov;
  //   ->  original version has a poor numerical stability, as its outcome is
  //   not guaranteed to be symmetric nor positive definite, thus make it to be
  //   a symmetric and positive Joseph form
  state_ptr_->cov = I_KH * P * I_KH.transpose() + K * V * K.transpose();
  state_update();
}

// void ESKF::update_odom(ODOMPtr odom_data_ptr) {
//   Eigen::Vector3d vel_odom = odom_data_ptr->linear_vel;
//   Eigen::Vector3d vel_world = state_ptr_->R_W_I * vel_odom;
//   Eigen::Matrix<double, 3, 18> H;
//   H.setZero();
//   H.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity();
//   Eigen::Matrix3d V = odom_noise_ * Eigen::Matrix3d::Identity();

//   // K = P * Ht * ( H * P * Ht + V).inverse();
//   // dx = K * (y - h(x))
//   // P = (I - KH) * P * (I - KH)t + K * V * Kt
//   const Eigen::MatrixXd &P = state_ptr_->cov;
//   const Eigen::MatrixXd K =
//       P * H.transpose() * (H * P * H.transpose() + V).inverse();
//   delta_x_ = K * (vel_world - state_ptr_->V_W_I);

//   const Eigen::MatrixXd I_KH =
//       Eigen::Matrix<double, 18, 18>::Identity() - K * H;
//   // state_ptr_->cov = I_KH * state_ptr_->cov;
//   //   ->  original version has a poor numerical stability, as its outcome is
//   //   not guaranteed to be symmetric nor positive definite, thus make it to
//   be
//       //   a symmetric and positive Joseph form
//       state_ptr_->cov = I_KH * P * I_KH.transpose() + K * V * K.transpose();
//   state_update();
// }

bool ESKF::process_ODOM_Data(ODOMPtr odom_data_ptr) {
  // if (!imu_initialized_) {
  //   if (imu_buffer_.size() < IMU_Buffer_Size) {
  //     ROS_WARN("Wait. Insufficient IMU data.");
  //     return false;
  //   }
  //   last_imu_ptr_ = imu_buffer_.back();
  //   if (std::abs(last_imu_ptr_->timestamp - odom_data_ptr->timestamp) > 0.1)
  //   {
  //     ROS_WARN("Odom and IMU are not sychonized.");
  //     return false;
  //   }
  //   if (!initialize()) return false;
  //   imu_initialized_ = true;
  // }
  // update_odom(odom_data_ptr);
  // return true;
  if (imu_initialized_ && gnss_initialized_) {
    update_odom(odom_data_ptr);
    return true;
  }
  return false;
}

void ESKF::state_update() {
  state_ptr_->P_W_I += delta_x_.block<3, 1>(0, 0);
  state_ptr_->V_W_I += delta_x_.block<3, 1>(3, 0);
  state_ptr_->R_W_I = state_ptr_->R_W_I * SO3d::exp(delta_x_.block<3, 1>(6, 0));
  state_ptr_->acc_bias += delta_x_.block<3, 1>(9, 0);
  state_ptr_->gyro_bias += delta_x_.block<3, 1>(12, 0);
  state_ptr_->g_W += delta_x_.block<3, 1>(15, 0);
  // update covariance and clear error state
  Eigen::Matrix<double, 18, 18> J = Eigen::Matrix<double, 18, 18>::Identity();
  J.block<3, 3>(6, 6) = Eigen::Matrix3d::Identity() -
                        0.5 * SO3d::hat(delta_x_.block<3, 1>(6, 0)).matrix();
  state_ptr_->cov = J * state_ptr_->cov * J.transpose();
  delta_x_.setZero();
}

bool ESKF::initialize() {
  Eigen::Vector3d sum_acc(0., 0., 0.);
  for (auto imu_data : imu_buffer_) {
    sum_acc += imu_data->acc;
  }
  const Eigen::Vector3d mean_acc = sum_acc / (double)imu_buffer_.size();

  Eigen::Vector3d sum_err2(0., 0., 0.);
  for (auto imu_data : imu_buffer_)
    sum_err2 += (imu_data->acc - mean_acc).cwiseAbs2();
  const Eigen::Vector3d std_acc =
      (sum_err2 / (double)imu_buffer_.size()).cwiseSqrt();
  if (std_acc.maxCoeff() > IMU_Std) {
    return false;
  }

  // z-axis
  const Eigen::Vector3d &z_axis = mean_acc.normalized();

  // x-axis
  Eigen::Vector3d x_axis =
      Eigen::Vector3d::UnitX() -
      z_axis * z_axis.transpose() * Eigen::Vector3d::UnitX();
  x_axis.normalize();

  // y-axis
  Eigen::Vector3d y_axis = z_axis.cross(x_axis);
  y_axis.normalize();

  Eigen::Matrix3d R_I_W;
  R_I_W.block<3, 1>(0, 0) = x_axis;
  R_I_W.block<3, 1>(0, 1) = y_axis;
  R_I_W.block<3, 1>(0, 2) = z_axis;
  SO3d so3_R_I_W(R_I_W);

  state_ptr_->R_W_I = so3_R_I_W.inverse();
  state_ptr_->timestamp = last_imu_ptr_->timestamp;
  state_ptr_->P_W_I.setZero();
  state_ptr_->V_W_I.setZero();
  state_ptr_->acc_bias.setZero();
  state_ptr_->gyro_bias.setZero();
  state_ptr_->g_W = -mean_acc / mean_acc.norm() * gravity_norm;
  state_ptr_->cov.setZero();
  state_ptr_->cov.block<3, 3>(0, 0) = 100. * Eigen::Matrix3d::Identity();
  state_ptr_->cov.block<3, 3>(3, 3) = 100. * Eigen::Matrix3d::Identity();
  state_ptr_->cov.block<2, 2>(6, 6) =
      10. * degree2radians * 10. * degree2radians * Eigen::Matrix2d::Identity();
  state_ptr_->cov(8, 8) = 100. * degree2radians * 100. * degree2radians;
  state_ptr_->cov.block<3, 3>(9, 9) = 0.0004 * Eigen::Matrix3d::Identity();
  state_ptr_->cov.block<3, 3>(12, 12) = 0.0004 * Eigen::Matrix3d::Identity();
  state_ptr_->cov.block<3, 3>(15, 15) = 0.0004 * Eigen::Matrix3d::Identity();

  return true;
}

}  // namespace AWV_Fusion