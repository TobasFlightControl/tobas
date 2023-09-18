#pragma once

#include <kdl/frames.hpp>

#include <dh_linear_control/lqr.hpp>

namespace tobas_mr_translation_lqr
{
struct Config
{
  double acc_delay_time_const;
  double hor_pos_weight;
  double ver_pos_weight;
  double hor_vel_weight;
  double ver_vel_weight;
  double acc_weight;
  double jerk_weight;
};

class Controller
{
  static constexpr uint32_t kPosIdx = 0;
  static constexpr uint32_t kVelIdx = kPosIdx + 3;
  static constexpr uint32_t kAccIdx = kVelIdx + 3;
  static constexpr uint32_t kStateSize = kAccIdx + 3;
  static constexpr uint32_t kInputSize = 3;

  static constexpr double kVerAccDecayTimeConst = 0.02;  // [s]

public:
  explicit Controller();

  void update(
    const KDL::Vector& cur_pos,
    const KDL::Vector& cur_vel,
    const KDL::Vector& cur_acc,
    const KDL::Vector& tar_pos,
    const double& dt,
    KDL::Vector& tar_acc);
  void configure(const Config& config);

private:
  ctrl::LQD lqd_;
};
}  // namespace tobas_mr_translation_lqr
