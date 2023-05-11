#include "../../include/tobas_tools/stability_derivatives_cog.hpp"

using namespace std;
using namespace KDL;

StabilityDerivativesCG::StabilityDerivativesCG(const Drone& drone)
  : drone_(drone), inertia_solver_(drone.tree())
{
  JntArray q_0(drone.tree().getNrOfJoints());
  update(q_0);
}

void StabilityDerivativesCG::update(const JntArray& q)
{
  // エイリアス
  const auto& fixed_wing = drone_.fixedWingConfig();
  const auto& vehicle = fixed_wing.vehicle;
  const auto& aero = fixed_wing.aerodynamics;
  const auto& control_surfaces = fixed_wing.control_surfaces;

  // CoGを更新
  inertia_solver_.JntToCart(q, cog_, I_);

  // 安定微係数を更新: (2.2-40), (3.2-23)
  const auto dx = vehicle.ac.x() - cog_.x();
  const auto dx_b = dx / vehicle.wing_span;
  const auto dx_c = dx / vehicle.mac;

  c_pitch_alpha_cg_ = aero.c_pitch_alpha + dx_c * aero.c_lift_alpha;
  c_yaw_beta_cg_ = aero.c_yaw_beta + dx_b * aero.c_side_beta;

  for (int i = 0; i < control_surfaces.size(); ++i)
  {
    const auto& cs = control_surfaces[i];
    c_pitch_delta_cg_[i] = cs.c_pitch_delta + dx_c * cs.c_lift_delta;
    c_yaw_delta_cg_[i] = cs.c_yaw_delta + dx_b * cs.c_side_delta;
  }
}

double StabilityDerivativesCG::cPitchAlpha() const
{
  return c_pitch_alpha_cg_;
}

double StabilityDerivativesCG::cYawBeta() const
{
  return c_yaw_beta_cg_;
}

double StabilityDerivativesCG::cPitchDelta(uint32_t cs_idx) const
{
  return c_pitch_delta_cg_[cs_idx];
}

double StabilityDerivativesCG::cYawDelta(uint32_t cs_idx) const
{
  return c_yaw_delta_cg_[cs_idx];
}
