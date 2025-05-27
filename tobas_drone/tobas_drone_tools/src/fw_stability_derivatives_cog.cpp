#include "tobas_drone_tools/fw_stability_derivatives_cog.hpp"

using namespace std;

namespace tobas
{
StabilityDerivativesCG::StabilityDerivativesCG(const Drone& drone, const kdl::Tree& tree)
  : drone_(drone), tree_(tree), inertia_solver_(tree)
{
}

bool StabilityDerivativesCG::updateInternalDataStructures()
{
  if (!inertia_solver_.updateInternalDataStructures()) {
    return false;
  }

  c_pitch_delta_cg_.clear();
  c_yaw_delta_cg_.clear();

  return true;
}

int StabilityDerivativesCG::update(const kdl::JntArray& q)
{
  error_code_ = E_NO_ERROR;

  // エイリアス
  const auto& aero = drone_.fixed_wing->aerodynamics;

  // CoGを更新
  if (inertia_solver_.JntToCart(q) < 0) {
    error_msg_ = inertia_solver_.errorMessage();
    return error_code_ = E_ERROR;
  }
  const auto cog = inertia_solver_.getInertia().getCOG();

  // 安定微係数を更新: (2.2-40), (3.2-23)
  const auto dx = drone_.fixed_wing->vehicle.ac.x() - cog.x();
  const auto dx_b = dx / drone_.fixed_wing->vehicle.wing_span;
  const auto dx_c = dx / drone_.fixed_wing->vehicle.mac;

  c_pitch_alpha_cg_ = aero.c_pitch_alpha + dx_c * aero.c_lift_alpha;
  c_yaw_beta_cg_ = aero.c_yaw_beta + dx_b * aero.c_side_beta;

  for (const auto& [channel, cs] : drone_.fixed_wing->control_surfaces) {
    c_pitch_delta_cg_[channel] = cs.c_pitch_delta + dx_c * cs.c_lift_delta;
    c_yaw_delta_cg_[channel] = cs.c_yaw_delta + dx_b * cs.c_side_delta;
  }

  return error_code_;
}
}  // namespace tobas
