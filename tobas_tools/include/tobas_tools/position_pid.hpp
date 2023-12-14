#pragma once

#include <dh_linear_control/pid3.hpp>

namespace tobas
{
struct PositionPidConfig
{
  double hor_kp;
  double hor_ki;
  double hor_kd;
  double ver_kp;
  double ver_ki;
  double ver_kd;

  double max_hor_acc;
  double max_ver_acc;
  double max_hor_acc_int;  // [m/s^2] I成分によって生成される加速度の水平成分の最大値
  double max_ver_acc_int;  // [m/s^2] I成分によって生成される加速度の垂直成分の最大値
};

class PositionPid
{
public:
  explicit PositionPid();

  Eigen::Vector3d update(
    const Eigen::Vector3d& cur_pos,
    const Eigen::Vector3d& cur_vel,
    const Eigen::Vector3d& tar_pos,
    const Eigen::Vector3d& tar_vel,
    const double& dt);

  void configure(const PositionPidConfig& cfg);

  inline const Eigen::Vector3d& integralError() const;

private:
  // Config
  double max_hor_acc_;
  double max_ver_acc_;

  ctrl::PID3 pid_;
};

inline const Eigen::Vector3d& PositionPid::integralError() const
{
  return pid_.integralError();
}
}  // namespace tobas
