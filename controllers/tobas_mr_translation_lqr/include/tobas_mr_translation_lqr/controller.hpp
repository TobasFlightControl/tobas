#pragma once

#include <kdl/frames.hpp>

#include <dh_linear_control/lqid.hpp>

namespace tobas_mr_translation_lqr
{
struct Config
{
  double acc_delay_time_const;
  double hor_pos_weight;
  double ver_pos_weight;
  double hor_vel_weight;
  double ver_vel_weight;
  double hor_acc_weight;
  double ver_acc_weight;
  double jerk_weight;
  double max_hor_pos_error;
  double max_ver_pos_error;
  double max_hor_vel;
  double max_ver_vel;
};

class VelocityController
{
  static constexpr uint32_t kVelSize = 3;
  static constexpr uint32_t kAccSize = 3;
  static constexpr uint32_t kVelIdx = 0;
  static constexpr uint32_t kAccIdx = kVelIdx + kVelSize;
  static constexpr uint32_t kStateSize = kAccIdx + kAccSize;
  static constexpr uint32_t kInputSize = kAccSize;
  static constexpr uint32_t kIntegrateSize = kVelSize;

  static constexpr double kVerAccDecayTimeConst = 0.02;  // [s]

public:
  explicit VelocityController();

  void update(
    const KDL::Vector& cur_vel,
    const KDL::Vector& cur_acc,
    const KDL::Vector& tar_vel,
    const double& dt,
    KDL::Vector& tar_acc);
  void configure(const Config& config);

private:
  double max_hor_vel_;
  double max_ver_vel_;

  ctrl::LQID lqid_;
};
}  // namespace tobas_mr_translation_lqr
