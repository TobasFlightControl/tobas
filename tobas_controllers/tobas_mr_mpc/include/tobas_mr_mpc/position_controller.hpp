#pragma once

#include <tobas_kdl/frames.hpp>
#include <tobas_linear_control/lqid.hpp>

namespace tobas_mr_mpc
{
struct PositionControllerConfig
{
  double acc_delay_time_const;

  double hor_pos_weight;
  double ver_pos_weight;
  double hor_vel_weight;
  double ver_vel_weight;
  double hor_acc_weight;
  double ver_acc_weight;
  double hor_posint_weight;
  double ver_posint_weight;
  int jerk_weight_log10;

  double max_hor_posint_error;
  double max_ver_posint_error;
  double max_hor_acc;
  double max_ver_acc;
};

class PositionController
{
  static constexpr size_t kPosIdx = 0;
  static constexpr size_t kVelIdx = kPosIdx + 3;
  static constexpr size_t kAccIdx = kVelIdx + 3;
  static constexpr size_t kStateSize = kAccIdx + 3;

  static constexpr double kVerAccDecayTimeConst = 0.02;  // [s]

public:
  explicit PositionController();

  void update(
    const kdl::Vector& cur_pos,
    const kdl::Vector& cur_vel_W,
    const kdl::Vector& cur_acc_w,
    const kdl::Vector& tar_pos,
    const kdl::Vector& tar_vel_W,
    const double& dt,
    kdl::Vector& tar_acc_W);

  void configure(const PositionControllerConfig& cfg);

  inline Eigen::Vector3d positionIntegralError() const;

private:
  double max_hor_acc_;
  double max_ver_acc_;

  ctrl::LQID lqid_;
};

inline Eigen::Vector3d PositionController::positionIntegralError() const
{
  return lqid_.integralError();
}
}  // namespace tobas_mr_mpc
