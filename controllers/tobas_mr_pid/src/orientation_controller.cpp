#include <cassert>

#include <dh_eigen_tools/geometry.hpp>

#include "../include/tobas_mr_pid/orientation_controller.hpp"

using namespace std;
using namespace KDL;

namespace tobas_mr_pid
{
OrientationController::OrientationController()
{
  gyro_lpf_.initialize(dh_std::timeConstFromCutoffFreq(kGyroLpfCutoff), Vector::Zero());
}

Vector OrientationController::update(
  const Euler& cur_rpy,
  const Vector& cur_gyro,
  const Euler& tar_rpy,
  const Vector& tar_gyro,
  const double& dt)
{
  assert(dt >= 0);

  // ジャイロノイズ (によるDジャイロのZ成分のノイズ) が回転数に大きく影響するためLPFに通す
  gyro_lpf_.update(cur_gyro, dt);

  // オイラー角で誤差を計算する場合
  Vector ep;
  ep.x(tar_rpy.roll - cur_rpy.roll);
  ep.y(tar_rpy.pitch - cur_rpy.pitch);
  ep.z(tar_rpy.yaw - cur_rpy.yaw);
  ei_ += ep * dt;

  const auto gyro_error = tar_gyro - gyro_lpf_.getState();
  const auto ed =
    Vector(eigen_tools::eulerrateFromAngvelLocal(gyro_error.data, cur_rpy.roll, cur_rpy.pitch));

  // 機体座標系から見た角軸ベクトルで誤差を計算する場合
  // Z成分にはロールピッチの誤差も含まれ，Z成分のゲインを下げると姿勢追従性能が低下してしまうためボツ
  // const auto ep = (cur_rpy.toRotation().inverse() * tar_rpy.toRotation()).GetRot();
  // ei_ += ep * dt;
  // const auto ed = tar_gyro - gyro_lpf_.getState();

  // PID
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
