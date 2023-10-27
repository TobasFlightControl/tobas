#pragma once

#include <dh_kdl/frames.hpp>

namespace tobas_mr_pid
{
struct TranslationControllerConfig
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

class TranslationController
{
public:
  explicit TranslationController();

  void update(
    const KDL::Vector& cur_pos,
    const KDL::Vector& cur_vel,
    const KDL::Vector& tar_pos,
    const KDL::Vector& tar_vel,
    KDL::Vector& tar_acc,
    const double& dt);
  void configure(const TranslationControllerConfig& cfg);

private:
  TranslationControllerConfig cfg_;
  KDL::Vector ei_ = KDL::Vector::Zero();  // 位置の積分誤差
};
}  // namespace tobas_mr_pid
