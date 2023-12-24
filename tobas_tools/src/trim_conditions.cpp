#include <tobas_std_tools/standard_atmosphere.hpp>
#include <tobas_std_tools/assert.hpp>

#include "../include/tobas_tools/trim_conditions.hpp"
#include "../include/tobas_tools/constants.hpp"

using namespace std;
using namespace KDL;

namespace tobas
{
TrimConditions::TrimConditions(const Drone& drone)
  : drone_(drone), inertia_solver_(drone.tree()), asd_cog_(drone)
{
  if (drone.isLoaded())
    updateInternalDataStructures();
}

void TrimConditions::updateInternalDataStructures()
{
  inertia_solver_.updateInternalDataStructures();
  asd_cog_.updateInternalDataStructures();

  if (inertia_solver_.JntToCart(JntArray::Zero(drone_.tree().getNrOfJoints())) < 0)
    throw runtime_error("Inertia solver failed: " + inertia_solver_.errorMessage());
  W_ = inertia_solver_.getInertia().getMass() * kGravity;

  setElevatorIndex();

  const auto& aero = drone_.aerodynamics();
  const auto& elev_cs = drone_.controlSurface(elev_idx_);

  const auto ml_raito = elev_cs.c_lift_delta / elev_cs.c_pitch_delta;
  a_ = aero.c_lift_alpha - aero.c_pitch_alpha * ml_raito;
  b_ = aero.c_lift_0 - aero.c_pitch_0 * ml_raito;
  assert(a_ > 0);
  assert(b_ > 0);
}

int TrimConditions::update(const double& V, const double& rho, const JntArray& q)
{
  assert(V > 0);
  assert(rho > 0);

  if (q.rows() != drone_.tree().getNrOfJoints())
  {
    error_msg_ = kErrorSizeMismatch;
    return -1;
  }

  const auto speed_limit = speedLimit(rho);
  if (!speed_limit.inRange(V))
  {
    error_msg_ = "V = " + to_string(V) + " is out of valid speed range " + speed_limit.toString();
    return -1;
  }

  // エイリアス
  const auto& aero = drone_.aerodynamics();
  const auto& elev_cs = drone_.controlSurface(elev_idx_);

  // CoGまわりの安定微係数
  if (asd_cog_.update(q) < 0)
  {
    error_msg_ = asd_cog_.errorMessage();
    return -1;
  }
  const auto c_pitch_alpha_cg = asd_cog_.cPitchAlpha();
  const auto c_pitch_elev_cg = asd_cog_.cPitchDelta(elev_idx_);
  if (c_pitch_elev_cg == 0)
  {
    error_msg_ = "The stability derivative of the elevator w.r.t. the pitch angle is zero";
    return -1;
  }

  // 引数に依存する定数
  const auto q_bar = dynamicPressure(rho, V);

  // 縦系の釣り合い
  c_L_ = W_ / (q_bar * drone_.vehicle().wing_surface);                          // (2.9-47)
  alpha_ = (c_L_ - b_) / a_;                                                    // (2.9-49)
  elevator_ = -(aero.c_pitch_0 + c_pitch_alpha_cg * alpha_) / c_pitch_elev_cg;  // (2.9-46)
  const auto c_D_alpha = aero.c_drag_0 + aero.c_drag_alpha * alpha_;  // TODO: 2次以上も考慮
  c_D_ = c_D_alpha + elev_cs.c_drag_abs_delta * abs(elevator_);       // (1.8-3)
  c_T_ = c_D_ / cos(alpha_);                                          // (2.2-10b)

  // その他依存変数
  u_ = V * cos(alpha_);

  if (alpha_ < -M_PI_2 || M_PI_2 < alpha_)
  {
    error_msg_ = "The angle of attack in the trimmed condition exceeds π/2";
    return -1;
  }
  if (!drone_.controlSurface(elev_idx_).angle_limit.inRange(elevator_))
  {
    error_msg_ = "The trim angle of the elevator is outside the range of the angle limit.";
    return -1;
  }

  return 0;
}

tobas_std::Range<double> TrimConditions::speedLimit(const double& rho) const
{
  assert(rho > 0);

  const auto c = 2 * W_ / rho / drone_.vehicle().wing_surface;

  // 迎角の最大値から最小速度を求める
  const auto max_den = a_ * drone_.vehicle().alpha_limit.upper + b_;
  assert(max_den > 0.);
  const auto V_min = sqrt(c / max_den);

  // 迎角の最小値から最大速度を求める
  // 分母が+0になる場合は，理論上無限の速度で水平飛行できる
  const auto min_den = a_ * drone_.vehicle().alpha_limit.lower + b_;
  const auto V_max = min_den > 0. ? sqrt(c / min_den) : numeric_limits<double>::max();

  return tobas_std::Range<double>(V_min, V_max);
}

double TrimConditions::takeOffSpeed(const double& rho) const
{
  assert(rho > 0);

  const auto c = 2 * W_ / rho / drone_.vehicle().wing_surface;
  constexpr double alpha_zero = 0.;
  return sqrt(c / (a_ * alpha_zero + b_));
}

void TrimConditions::setElevatorIndex()
{
  assert(drone_.numControlSurfaces() > 0);

  double max_c_pitch_delta = numeric_limits<double>::lowest();
  for (size_t cs_idx = 0; cs_idx < drone_.numControlSurfaces(); ++cs_idx)
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
