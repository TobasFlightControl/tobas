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
  double max_hor_acc_int;  // [m/s^2] I成分によって生成される加速度の水平成分の最大値
  double max_ver_acc_int;  // [m/s^2] I成分によって生成される加速度の垂直成分の最大値
};

class PositionController
{
public:
  explicit PositionController();

  KDL::Vector update(
    const KDL::Vector& cur_pos,
    const KDL::Vector& cur_vel,
    const KDL::Vector& tar_pos,
    const KDL::Vector& tar_vel,
    const double& dt);

  void configure(const PositionControllerConfig& cfg);

  inline const KDL::Vector& integralError() const;

private:
  // Config
  KDL::Vector kp_;
  KDL::Vector ki_;
  KDL::Vector kd_;
  double max_hor_acc_;
  double max_ver_acc_;
  double max_hor_int_err_;  // [ms] 積分誤差の水平成分の最大値
  double max_ver_int_err_;  // [ms] 積分誤差の垂直成分の最大値

  KDL::Vector ei_ = KDL::Vector::Zero();  // [ms] 位置の積分誤差
};

inline const KDL::Vector& PositionController::integralError() const
{
  return ei_;
}
}  // namespace tobas_mr_pid
