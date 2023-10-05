#include "../include/tobas_tools/stability_derivatives_cog.hpp"

using namespace std;
using namespace KDL;

namespace tobas
{
StabilityDerivativesCG::StabilityDerivativesCG(const Drone& drone)
  : drone_(drone), inertia_solver_(drone.tree())
{
  if (drone.isLoaded())
  {
    updateInternalDataStructures();
  }
}

void StabilityDerivativesCG::updateInternalDataStructures()
{
  inertia_solver_.updateInternalDataStructures();

  c_pitch_delta_cg_.resize(drone_.numControlSurfaces());
  c_yaw_delta_cg_.resize(drone_.numControlSurfaces());
}

void StabilityDerivativesCG::update(const JntArray& q)
{
  // エイリアス
  const auto& aero = drone_.aerodynamics();

  // CoGを更新
  const auto I_base = inertia_solver_.JntToCart(q);
  const auto cog = I_base.getCOG();

  // 安定微係数を更新: (2.2-40), (3.2-23)
  const auto dx = drone_.vehicle().ac.x() - cog.x();
  const auto dx_b = dx / drone_.vehicle().wing_span;
  const auto dx_c = dx / drone_.vehicle().mac;

  c_pitch_alpha_cg_ = aero.c_pitch_alpha + dx_c * aero.c_lift_alpha;
  c_yaw_beta_cg_ = aero.c_yaw_beta + dx_b * aero.c_side_beta;

  for (uint32_t i = 0; i < drone_.numControlSurfaces(); ++i)
  {
    const auto& cs = drone_.controlSurface(i);
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

double StabilityDerivativesCG::cPitchDelta(const uint32_t& cs_idx) const
{
  return c_pitch_delta_cg_[cs_idx];
}

double StabilityDerivativesCG::cYawDelta(const uint32_t& cs_idx) const
{
  return c_yaw_delta_cg_[cs_idx];
}
}  // namespace tobas
