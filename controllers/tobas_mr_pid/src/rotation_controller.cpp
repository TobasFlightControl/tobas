#include <cassert>
#include <algorithm>

#include <dh_eigen_tools/geometry.hpp>

#include "../include/tobas_mr_pid/rotation_controller.hpp"

using namespace std;
using namespace Eigen;

namespace tobas_mr_pid
{
RotationController::RotationController()
{
}

Vector3d RotationController::update(
  const Vector3d& cur_pos,
  const Vector3d& cur_vel,
  const Vector3d& tar_pos,
  const Vector3d& tar_vel,
  const double& dt)
{
  assert(dt >= 0);

  // 誤差を計算
  const auto ep = tar_pos - cur_pos;
  ei_ += (tar_pos - cur_pos) * dt;
  const auto ed = tar_vel - cur_vel;

  // 目標オイラー加速度を計算
  Vector3d tar_ddrpy;
  tar_ddrpy.x() = config_.atti_kp * ep.x() + config_.atti_ki * ei_.x() + config_.atti_kd * ed.x();
  tar_ddrpy.y() = config_.atti_kp * ep.y() + config_.atti_ki * ei_.y() + config_.atti_kd * ed.y();
  tar_ddrpy.z() = config_.head_kp * ep.z() + config_.head_ki * ei_.z() + config_.head_kd * ed.z();

  // オイラー加速度を角加速度に変換
  const Vector3d tar_dgyro =
    eigen_tools::angaccFromEuleraccLocal(cur_pos.x(), cur_pos.y(), cur_vel, tar_ddrpy);
  return tar_dgyro;
}

void RotationController::configure(const RotationControllerConfig& config)
{
  assert(config.atti_kp >= 0);
  assert(config.atti_ki >= 0);
  assert(config.atti_kd >= 0);
  assert(config.head_kp >= 0);
  assert(config.head_ki >= 0);
  assert(config.head_kd >= 0);

  config_ = config;
}
}  // namespace tobas_mr_pid
