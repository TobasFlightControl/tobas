#pragma once

#include <dh_std_tools/first_order_filter.hpp>
#include <dh_kdl/euler.hpp>

namespace tobas_mr_pid
{
struct OrientationControllerConfig
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

class OrientationController
{
  // Constants
  static constexpr uint32_t kGyroLpfCutoff = 20;

public:
  explicit OrientationController();

  KDL::Vector update(
    const KDL::Euler& cur_rpy,
    const KDL::Vector& cur_gyro,
    const KDL::Euler& tar_rpy,
    const KDL::Vector& tar_gyro,
    const double& dt);

  void configure(const OrientationControllerConfig& cfg);

private:
  // Config
  KDL::Vector kp_;
  KDL::Vector ki_;
  KDL::Vector kd_;
  double max_atti_int_err_;  // [rads] 積分誤差の姿勢成分の最大値
  double max_head_int_err_;  // [rads] 積分誤差の方位成分の最大値

  dh_std::FirstOrderFilter<KDL::Vector> gyro_lpf_;

  KDL::Vector ei_ = KDL::Vector::Zero();
};
}  // namespace tobas_mr_pid
