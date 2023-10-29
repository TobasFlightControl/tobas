#pragma once

#include <dh_kdl/frames.hpp>

namespace tobas_mr_pid
{
struct PositionControllerConfig
{
  double hor_kp;
  double hor_ki;
  double hor_kd;
  double ver_kp;
  double ver_ki;
  double ver_kd;

  double max_hor_acc;
  double max_ver_acc;
};

class PositionController
{
public:
  explicit PositionController();

  void update(
    const KDL::Vector& cur_pos,
    const KDL::Vector& cur_vel,
    const KDL::Vector& tar_pos,
    const KDL::Vector& tar_vel,
    KDL::Vector& tar_acc,
    const double& dt);
  void configure(const PositionControllerConfig& cfg);

  inline const KDL::Vector& integralError() const;

private:
  PositionControllerConfig cfg_;
  KDL::Vector ei_ = KDL::Vector::Zero();  // [ms] 位置の積分誤差
};

inline const KDL::Vector& PositionController::integralError() const
{
  return ei_;
}
}  // namespace tobas_mr_pid
