#include <dh_std_tools/standard_atmosphere.hpp>

#include "../../include/tobas_tools/trim_conditions.hpp"
#include "../../include/tobas_tools/constants.hpp"

using namespace std;
using namespace KDL;

namespace tobas
{
TrimConditions::TrimConditions(const Drone& drone)
  : drone_(drone), inertia_solver_(drone.tree()), asd_cog_(drone)
{
  if (drone.isLoaded())
  {
    updateInternalDataStructures();
  }
}

void TrimConditions::updateInternalDataStructures()
{
  inertia_solver_.updateInternalDataStructures();
  asd_cog_.updateInternalDataStructures();

  W_ = inertia_solver_.JntToMass() * kGravity;
  setElevatorIndex();

  const auto& aero = drone_.aerodynamics();
  const auto& elev_cs = drone_.controlSurface(elev_idx_);

  const auto ml_raito = elev_cs.c_lift_delta / elev_cs.c_pitch_delta;
  a_ = aero.c_lift_alpha - aero.c_pitch_alpha * ml_raito;
  b_ = aero.c_lift_0 - aero.c_pitch_0 * ml_raito;
  assert(a_ > 0.);
}

void TrimConditions::update(double V, double rho, const JntArray& q)
{
  assert(V > 0.);
  assert(rho > 0.);
  assert(speedLimit(rho).inRange(V));
  assert(q.rows() == drone_.tree().getNrOfJoints());

  // エイリアス
  const auto& aero = drone_.aerodynamics();
  const auto& elev_cs = drone_.controlSurface(elev_idx_);

  // CoGまわりの安定微係数
  asd_cog_.update(q);
  const auto c_pitch_alpha_cg = asd_cog_.cPitchAlpha();
  const auto c_pitch_elev_cg = asd_cog_.cPitchDelta(elev_idx_);
  assert(c_pitch_elev_cg != 0.);

  // 引数に依存する定数
  const auto q_bar = dynamicPressure(rho, V);

  // 縦系の釣り合い
  c_L_ = W_ / (q_bar * drone_.vehicle().wing_surface);                          // (2.9-47)
  alpha_ = (c_L_ - b_) / a_;                                                    // (2.9-49)
  elevator_ = -(aero.c_pitch_0 + c_pitch_alpha_cg * alpha_) / c_pitch_elev_cg;  // (2.9-46)
  const auto c_D_alpha = aero.c_drag_0 + aero.c_drag_alpha * alpha_;  // TODO: 2次以上も考慮
  c_D_ = c_D_alpha + elev_cs.c_drag_abs_delta * elevator_;            // (1.8-3)
  c_T_ = c_D_ / cos(alpha_);                                          // (2.2-10b)
  assert(-M_PI_2 < alpha_ && alpha_ < M_PI_2);
  assert(drone_.controlSurface(elev_idx_).angle_limit.inRange(elevator_));

  // その他依存変数
  u_ = V * cos(alpha_);
}

const StabilityDerivativesCG& TrimConditions::stabilityDerivativesCG() const
{
  return asd_cog_;
}

const uint32_t& TrimConditions::elevatorIndex() const
{
  return elev_idx_;
}

const double& TrimConditions::alpha() const
{
  return alpha_;
}

const double& TrimConditions::theta() const
{
  return alpha_;  // 水平飛行より theta = alpha
}

const double& TrimConditions::elevator() const
{
  return elevator_;
}

const double& TrimConditions::c_L() const
{
  return c_L_;
}

const double& TrimConditions::c_D() const
{
  return c_D_;
}

const double& TrimConditions::c_T() const
{
  return c_L_;
}

const double& TrimConditions::u() const
{
  return u_;
}

dh_std::Range<double> TrimConditions::speedLimit(double rho) const
{
  const auto c = 2. * W_ / rho / drone_.vehicle().wing_surface;

  // 迎角の最大値から最小速度を求める
  const auto max_den = a_ * drone_.vehicle().alpha_limit.upper + b_;
  assert(max_den > 0.);
  const auto V_min = sqrt(c / max_den);

  // 迎角の最小値から最大速度を求める
  // 分母が+0になる場合は，理論上無限の速度で水平飛行できる
  const auto min_den = a_ * drone_.vehicle().alpha_limit.lower + b_;
  const auto V_max = min_den > 0. ? sqrt(c / min_den) : numeric_limits<double>::max();

  return dh_std::Range<double>(V_min, V_max);
}

void TrimConditions::setElevatorIndex()
{
  double max_c_pitch_delta = -1.;
  for (int cs_idx = 0; cs_idx < drone_.numControlSurfaces(); ++cs_idx)
  {
    const auto& cs = drone_.controlSurface(cs_idx);
    if (abs(cs.c_pitch_delta) > max_c_pitch_delta)
    {
      max_c_pitch_delta = abs(cs.c_pitch_delta);
      elev_idx_ = cs_idx;
    }
  }
}
}  // namespace tobas
