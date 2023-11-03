#pragma once

#include <dh_std_tools/first_order_filter.hpp>
#include <dh_linear_control/pid3.hpp>
#include <dh_kdl/euler.hpp>

namespace tobas
{
struct OrientationPidConfig
{
  double atti_kp;
  double atti_ki;
  double atti_kd;
  double head_kp;
  double head_ki;
  double head_kd;

  double max_atti_acc_int;  // [rad/s^2] I成分によって生成される角加速度の姿勢成分の最大値
  double max_head_acc_int;  // [rad/s^2] I成分によって生成される角加速度の方位成分の最大値
};

class OrientationPid
{
  // Constants
  static constexpr uint32_t kGyroLpfCutoff = 20;

public:
  explicit OrientationPid();

  KDL::Vector update(
    const KDL::Euler& cur_rpy,
    const KDL::Vector& cur_gyro,
    const KDL::Euler& tar_rpy,
    const KDL::Vector& tar_gyro,
    const double& dt);

  void configure(const OrientationPidConfig& cfg);

  inline KDL::Vector integralError() const;

private:
  dh_std::FirstOrderFilter<KDL::Vector> gyro_lpf_;
  ctrl::PID3 pid_;
};

inline KDL::Vector OrientationPid::integralError() const
{
  return KDL::Vector(pid_.integralError());
}
}  // namespace tobas_mr_pid
