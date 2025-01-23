#ifndef AWV_FUSION_UTILS_H
#define AWV_FUSION_UTILS_H

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <GeographicLib/LocalCartesian.hpp>
#include <cfloat>
#include <deque>
#include <fstream>
#include <iostream>
#include <memory>
#include "sophus/se2.hpp"
#include "sophus/se3.hpp"
#include "sophus/so3.hpp"

namespace AWV_Fusion {

constexpr double degree2radians = M_PI / 180.;
constexpr double radians2degree = 180. / M_PI;
constexpr double gravity_norm = 9.81007;
constexpr int IMU_Buffer_Size = 50;
constexpr double IMU_Std = 3.0;

using SO3d = Sophus::SO3d;
using SE3d = Sophus::SE3d;

struct IMU {
  double timestamp;

  Eigen::Vector3d acc;
  Eigen::Vector3d gyro;
};
using IMUPtr = std::shared_ptr<IMU>;

struct GNSS {
  double timestamp;

  Eigen::Vector3d lla;
  Eigen::Matrix3d cov;
};
using GNSSPtr = std::shared_ptr<GNSS>;

struct ODOM {
  double timestamp;

  Eigen::Vector3d pose;
  Eigen::Quaterniond orientation;
  Eigen::Vector3d linear_vel;
  Eigen::Vector3d ang_vel;
  Eigen::Matrix3d cov;
};
using ODOMPtr = std::shared_ptr<ODOM>;

struct State {
  double timestamp;

  Eigen::Vector3d P_W_I;
  Eigen::Vector3d V_W_I;
  SO3d R_W_I;
  Eigen::Vector3d acc_bias;
  Eigen::Vector3d gyro_bias;
  Eigen::Vector3d g_W;

  Eigen::Matrix<double, 18, 18> cov;
};
using StatePtr = std::shared_ptr<State>;

inline void convert_lla_to_enu(const Eigen::Vector3d& init_lla,
                               const Eigen::Vector3d& point_lla,
                               Eigen::Vector3d* point_enu) {
  static GeographicLib::LocalCartesian local_cartesian;
  local_cartesian.Reset(init_lla(0), init_lla(1), init_lla(2));
  local_cartesian.Forward(point_lla(0), point_lla(1), point_lla(2),
                          point_enu->data()[0], point_enu->data()[1],
                          point_enu->data()[2]);
}

inline void convert_enu_to_lla(const Eigen::Vector3d& init_lla,
                               const Eigen::Vector3d& point_enu,
                               Eigen::Vector3d* point_lla) {
  static GeographicLib::LocalCartesian local_cartesian;
  local_cartesian.Reset(init_lla(0), init_lla(1), init_lla(2));
  local_cartesian.Reverse(point_enu(0), point_enu(1), point_enu(2),
                          point_lla->data()[0], point_lla->data()[1],
                          point_lla->data()[2]);
}

}  // namespace AWV_Fusion

#endif  // AWV_FUSION_UTILS_H
