#include <cassert>
#include <algorithm>

#include <dh_eigen_tools/geometry.hpp>

#include "../include/tobas_mr_pid/orientation_controller.hpp"

using namespace std;
using namespace Eigen;

namespace tobas_mr_pid
{
OrientationController::OrientationController()
{
}

Vector3d OrientationController::update(
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
  tar_ddrpy.x() = cfg_.atti_kp * ep.x() + cfg_.atti_ki * ei_.x() + cfg_.atti_kd * ed.x();
  tar_ddrpy.y() = cfg_.atti_kp * ep.y() + cfg_.atti_ki * ei_.y() + cfg_.atti_kd * ed.y();
  tar_ddrpy.z() = cfg_.head_kp * ep.z() + cfg_.head_ki * ei_.z() + cfg_.head_kd * ed.z();

  // オイラー加速度を角加速度に変換
  const Vector3d tar_dgyro =
    eigen_tools::angaccFromEuleraccLocal(cur_pos.x(), cur_pos.y(), cur_vel, tar_ddrpy);
  return tar_dgyro;
}

void OrientationController::configure(const OrientationControllerConfig& cfg)
{
  assert(cfg.atti_kp >= 0);
  assert(cfg.atti_ki >= 0);
  assert(cfg.atti_kd >= 0);
  assert(cfg.head_kp >= 0);
  assert(cfg.head_ki >= 0);
  assert(cfg.head_kd >= 0);

  cfg_ = cfg;
}
}  // namespace tobas_mr_pid
