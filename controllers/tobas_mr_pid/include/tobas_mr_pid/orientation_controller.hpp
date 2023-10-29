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

  dh_std::FirstOrderFilter<KDL::Vector> gyro_lpf_;

  KDL::Vector ei_ = KDL::Vector::Zero();
};
}  // namespace tobas_mr_pid
