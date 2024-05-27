#include <tobas_std_tools/standard_atmosphere.hpp>
#include <tobas_std_tools/assert.hpp>

#include "../include/tobas_tools/trim_conditions.hpp"
#include "../include/tobas_tools/constants.hpp"

using namespace std;
using namespace tobas_kdl;

namespace tobas
{
TrimConditions::TrimConditions(const Drone& drone) : drone_(drone), inertia_solver_(drone.tree()), asd_cog_(drone)
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

int TrimConditions::update(double V, const double& rho, const JntArray& q)
{
  assert(V > 0);
  assert(rho > 0);

  error_code_ = E_NO_ERROR;

  if (q.rows() != drone_.tree().getNrOfJoints())
  {
    error_msg_ = kErrorSizeMismatch;
    return error_code_ = E_ERROR;
  }

  // 速度が有効な範囲内にあるかチェック
  const auto speed_limit = speedLimit(rho);
  if (V < speed_limit.lower)
  {
    if (error_code_ > E_WARN)
    {
      error_msg_ = "Speed is too low: " + to_string(V) + " < " + to_string(speed_limit.lower);
      error_code_ = E_WARN;
    }
    V = speed_limit.lower;
  }
  else if (V > speed_limit.upper)
  {
    if (error_code_ > E_WARN)
    {
      error_msg_ = "Speed is too high: " + to_string(V) + " > " + to_string(speed_limit.upper);
      error_code_ = E_WARN;
    }
    V = speed_limit.upper;
  }

  // エイリアス
  const auto& aero = drone_.aerodynamics();
  const auto& elev_cs = drone_.controlSurface(elev_idx_);

  // CoGまわりの安定微係数
  asd_cog_.update(q);
  if (updateError(asd_cog_) <= E_ERROR)
    return error_code_;
  const auto& c_pitch_alpha_cg = asd_cog_.cPitchAlpha();
  const auto& c_pitch_elev_cg = asd_cog_.cPitchDelta(elev_idx_);
  if (c_pitch_elev_cg == 0)
  {
    error_msg_ = "The stability derivative of the elevator w.r.t. the pitch angle is zero.";
    return error_code_ = E_ERROR;
  }

  // 引数に依存する定数
  const auto q_bar = dynamicPressure(rho, V);

  // 縦系の釣り合い
  c_L_ = W_ / (q_bar * drone_.vehicle().wing_surface);                          // (2.9-47)
  alpha_ = (c_L_ - b_) / a_;                                                    // (2.9-49)
  elevator_ = -(aero.c_pitch_0 + c_pitch_alpha_cg * alpha_) / c_pitch_elev_cg;  // (2.9-46)
  const auto c_D_alpha = aero.c_drag_0 + aero.c_drag_alpha * alpha_;            // TODO: 2次以上も考慮
  c_D_ = c_D_alpha + elev_cs.c_drag_abs_delta * abs(elevator_);                 // (1.8-3)
  c_T_ = c_D_ / cos(alpha_);                                                    // (2.2-10b)

  // その他依存変数
  u_ = V * cos(alpha_);

  if (!drone_.vehicle().alpha_limit.inRange(alpha_))
  {
    if (error_code_ > E_WARN)
    {
      error_msg_ = "The angle of attack in the trimmed condition is outside the valid range.";
      error_code_ = E_WARN;
    }
    alpha_ = drone_.vehicle().alpha_limit.clamp(alpha_);
  }
  if (!drone_.controlSurface(elev_idx_).angle_limit.inRange(elevator_))
  {
    if (error_code_ > E_WARN)
    {
      error_msg_ = "The trim angle of the elevator is outside the range of the angle limit.";
      error_code_ = E_WARN;
    }
    elevator_ = drone_.controlSurface(elev_idx_).angle_limit.clamp(elevator_);
  }

  return error_code_;
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
