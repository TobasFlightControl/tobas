#include "../../include/tobas_control/pid/pid3.hpp"

using namespace std;
using namespace Eigen;

namespace ctrl
{
PID3::PID3()
{
}

Vector3d PID3::update(
  const Vector3d& cur_pos,
  const Vector3d& cur_vel,
  const Vector3d& tar_pos,
  const Vector3d& tar_vel,
  const double& dt)
{
  assert(dt >= 0);

  const Vector3d ep = tar_pos - cur_pos;
  const Vector3d ed = tar_vel - cur_vel;

  // 積分誤差を蓄積
  ei_ += ep * dt;

  // アンチワインドアップ
  ei_ = ei_.cwiseMax(-i_max).cwiseMin(i_max);

  // PID
  return kp.cwiseProduct(ep) + ki.cwiseProduct(ei_) + kd.cwiseProduct(ed);
}
}  // namespace ctrl
