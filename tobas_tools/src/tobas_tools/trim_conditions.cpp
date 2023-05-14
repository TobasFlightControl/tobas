#include <dh_std_tools/standard_atmosphere.hpp>

#include "../../include/tobas_tools/trim_conditions.hpp"
#include "../../include/tobas_tools/constants.hpp"

using namespace std;
using namespace KDL;

namespace tobas
{
TrimConditions::TrimConditions(const Drone& drone, uint32_t elev_cs_idx)
  : drone_(drone), elev_cs_idx_(elev_cs_idx), inertia_solver_(drone.tree()), asd_cog_(drone)
{
  W_ = inertia_solver_.JntToMass() * kGravity;

  const auto& fixed_wing = drone_.fixedWingConfig();
  const auto& aero = fixed_wing.aerodynamics;
  const auto& elev_cs = fixed_wing.control_surfaces[elev_cs_idx_];

  const auto ml_raito = elev_cs.c_lift_delta / elev_cs.c_pitch_delta;
  a_ = aero.c_lift_alpha - aero.c_pitch_alpha * ml_raito;
  b_ = aero.c_lift_0 - aero.c_pitch_0 * ml_raito;
}

void TrimConditions::update(double V, double h, const JntArray& q)
{
  assert(h > 0.);
  assert(speedLimit(h).inRange(V));
  assert(q.rows() == drone_.tree().getNrOfJoints());

  // エイリアス
  const auto& fixed_wing = drone_.fixedWingConfig();
  const auto& vehicle = fixed_wing.vehicle;
  const auto& aero = fixed_wing.aerodynamics;
  const auto& elev_cs = fixed_wing.control_surfaces[elev_cs_idx_];

  // CoGまわりの安定微係数
  asd_cog_.update(q);
  const auto c_pitch_alpha_cg = asd_cog_.cPitchAlpha();
  const auto c_pitch_elev_cg = asd_cog_.cPitchDelta(elev_cs_idx_);

  // 引数に依存する定数
  const auto rho = dh_std::altitudeToDensity(h);
  const auto q_bar = dynamicPressure(rho, V);

  // 縦系の釣り合い
  c_L_ = W_ / (q_bar * vehicle.wing_surface);                                   // (2.9-47)
  alpha_ = (c_L_ - b_) / a_;                                                    // (2.9-49)
  elevator_ = -(aero.c_pitch_0 + c_pitch_alpha_cg * alpha_) / c_pitch_elev_cg;  // (2.9-46)
  const auto c_D_alpha = aero.c_drag_0 + aero.c_drag_alpha * alpha_;  // TODO: 2次以上も考慮
  c_D_ = c_D_alpha + elev_cs.c_drag_abs_delta * elevator_;            // (1.8-3)
  c_T_ = c_D_ / cos(alpha_);                                          // (2.2-10b)

  // その他依存変数
  u_ = V * cos(alpha_);
}

const StabilityDerivativesCG& TrimConditions::stabilityDerivativesCG() const
{
  return asd_cog_;
}

const uint32_t& TrimConditions::elevatorIndex() const
{
  return elev_cs_idx_;
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

dh_std::Range<double> TrimConditions::speedLimit(double altitude) const
{
  const auto& vehicle = drone_.fixedWingConfig().vehicle;
  const auto& alpha_limit = vehicle.alpha_limit;

  const auto rho = dh_std::altitudeToDensity(altitude);
  const auto c = 2. * W_ / rho / vehicle.wing_surface;
  const auto V_min = sqrt(c / (a_ * alpha_limit.upper + b_));
  const auto V_max = sqrt(c / (a_ * alpha_limit.lower + b_));

  return dh_std::Range<double>(V_min, V_max);
}
}  // namespace tobas
