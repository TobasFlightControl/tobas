#include "../../include/tobas_fixed_wing_controller/dynamics.hpp"

using namespace std;
using namespace Eigen;

namespace tobas_fixed_wing_controller
{
FixedWingMicroDisturbanceDynamics::FixedWingMicroDisturbanceDynamics(const Drone& drone)
  : drone_(drone), kdl_model_(drone.tree())
{
  // TODO
}

void FixedWingMicroDisturbanceDynamics::update(double V)
{
  assert(V > 0.);

  // TODO
}

const FixedWingMicroDisturbanceDynamics::StateVector&
FixedWingMicroDisturbanceDynamics::trimState() const
{
  return trim_state_;
}

const VectorXd& FixedWingMicroDisturbanceDynamics::trimInput() const
{
  return trim_input_;
}

double FixedWingMicroDisturbanceDynamics::trimState_u() const
{
  return trim_state_(stateIdx_u);
}

double FixedWingMicroDisturbanceDynamics::trimState_alpha() const
{
  return trim_state_(stateIdx_alpha);
}

double FixedWingMicroDisturbanceDynamics::trimState_beta() const
{
  return trim_state_(stateIdx_beta);
}

double FixedWingMicroDisturbanceDynamics::trimState_phi() const
{
  return trim_state_(stateIdx_phi);
}

double FixedWingMicroDisturbanceDynamics::trimState_theta() const
{
  return trim_state_(stateIdx_theta);
}

double FixedWingMicroDisturbanceDynamics::trimState_p() const
{
  return trim_state_(stateIdx_p);
}

double FixedWingMicroDisturbanceDynamics::trimState_q() const
{
  return trim_state_(stateIdx_q);
}

double FixedWingMicroDisturbanceDynamics::trimState_r() const
{
  return trim_state_(stateIdx_r);
}
}  // namespace tobas_fixed_wing_controller
