#include <cassert>

#include <tobas_std_tools/algorithm.hpp>
#include <tobas_std_tools/check.hpp>
#include <tobas_eigen_tools/geometry.hpp>

#include "../include/tobas_tools/orientation_pid.hpp"

using namespace std;
using namespace Eigen;
using namespace KDL;

namespace tobas
{
OrientationPid::OrientationPid()
{
  gyro_lpf_.initialize(tobas_std::timeConstFromCutoffFreq(kGyroLpfCutoff), Vector::Zero());
}

Vector OrientationPid::update(
  const Euler& cur_rpy,
  const Vector& cur_gyro,
  const Euler& tar_rpy,
  const Vector& tar_gyro,
  const double& dt)
{
  // ジャイロノイズ (によるDジャイロのZ成分のノイズ) が回転数に大きく影響するためLPFに通す
  gyro_lpf_.update(cur_gyro, dt);

  // オイラー角で誤差を計算する場合
  const Vector3d ep(
    tar_rpy.roll - cur_rpy.roll, tar_rpy.pitch - cur_rpy.pitch, tar_rpy.yaw - cur_rpy.yaw);
  const Vector gyro_error = tar_gyro - gyro_lpf_.getState();
  const Vector3d ed =
    eigen_tools::eulerrateFromAngvelLocal(gyro_error.data, cur_rpy.roll, cur_rpy.pitch);

  // 機体座標系から見た角軸ベクトルで誤差を計算する場合
  // Z成分にはロールピッチの誤差も含まれ，Z成分のゲインを下げると姿勢追従性能が低下してしまうためボツ
  // const auto ep = (cur_rpy.toRotation().inverse() * tar_rpy.toRotation()).getRot();
  // const auto ed = tar_gyro - gyro_lpf_.getState();

  // 目標角加速度を計算
  return Vector(pid_.update(ep, ed, dt));
}

void OrientationPid::configure(const OrientationPidConfig& cfg)
{
  CHECK(cfg.atti_kp >= 0);
  CHECK(cfg.atti_ki >= 0);
  CHECK(cfg.atti_kd >= 0);
  CHECK(cfg.head_kp >= 0);
  CHECK(cfg.head_ki >= 0);
  CHECK(cfg.head_kd >= 0);
  CHECK(cfg.max_atti_acc_int >= 0);
  CHECK(cfg.max_head_acc_int >= 0);

  pid_.kp.x() = cfg.atti_kp;
  pid_.kp.y() = cfg.atti_kp;
  pid_.kp.z() = cfg.head_kp;
  pid_.ki.x() = cfg.atti_ki;
  pid_.ki.y() = cfg.atti_ki;
  pid_.ki.z() = cfg.head_ki;
  pid_.kd.x() = cfg.atti_kd;
  pid_.kd.y() = cfg.atti_kd;
  pid_.kd.z() = cfg.head_kd;

  const double max_atti_int_err = cfg.atti_ki > 0 ? cfg.max_atti_acc_int / cfg.atti_ki : 0;
  const double max_head_int_err = cfg.head_ki > 0 ? cfg.max_head_acc_int / cfg.head_ki : 0;
  pid_.i_max.x() = max_atti_int_err;
  pid_.i_max.y() = max_atti_int_err;
  pid_.i_max.z() = max_head_int_err;
}
}  // namespace tobas
