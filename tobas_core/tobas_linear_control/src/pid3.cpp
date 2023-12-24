#include "../include/tobas_linear_control/pid3.hpp"

using namespace std;
using namespace Eigen;

namespace ctrl
{
PID3::PID3()
{
  reset();
}

Vector3d PID3::update(const Vector3d& ep, const Vector3d& ed, const double& dt)
{
  assert(dt >= 0);

  // 積分誤差を蓄積
  ei_ += ep * dt;

  // アンチワインドアップ
  ei_ = ei_.cwiseMax(-i_max).cwiseMin(i_max);

  // PID
  return kp.cwiseProduct(ep) + ki.cwiseProduct(ei_) + kd.cwiseProduct(ed);
}
}  // namespace ctrl
