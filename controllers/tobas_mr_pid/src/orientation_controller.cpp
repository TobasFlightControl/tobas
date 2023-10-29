#include <cassert>

#include "../include/tobas_mr_pid/orientation_controller.hpp"

using namespace std;
using namespace KDL;

namespace tobas_mr_pid
{
OrientationController::OrientationController()
{
}

Vector OrientationController::update(
  const Euler& cur_rpy,
  const Vector& cur_gyro,
  const Euler& tar_rpy,
  const Vector& tar_gyro,
  const double& dt)
{
  assert(dt >= 0);

  // オイラー角ではなく，機体座標系から見た角軸ベクトル空間で誤差を計算
  const auto ep = (cur_rpy.toRotation().inverse() * tar_rpy.toRotation()).GetRot();
  ei_ += ep * dt;
  const auto ed = tar_gyro - cur_gyro;

  // PIDで角加速度を計算
  return kp_.hadamard(ep) + ki_.hadamard(ei_) + kd_.hadamard(ed);
}

void OrientationController::configure(const OrientationControllerConfig& cfg)
{
  assert(cfg.atti_kp >= 0);
  assert(cfg.atti_ki >= 0);
  assert(cfg.atti_kd >= 0);
  assert(cfg.head_kp >= 0);
  assert(cfg.head_ki >= 0);
  assert(cfg.head_kd >= 0);

  kp_.x(cfg.atti_kp);
  kp_.y(cfg.atti_kp);
  kp_.z(cfg.head_kp);
  ki_.x(cfg.atti_ki);
  ki_.y(cfg.atti_ki);
  ki_.z(cfg.head_ki);
  kd_.x(cfg.atti_kd);
  kd_.y(cfg.atti_kd);
  kd_.z(cfg.head_kd);
}
}  // namespace tobas_mr_pid
