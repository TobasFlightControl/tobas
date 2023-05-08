#include "../../include/tobas_fixed_wing_controller/dynamics.hpp"

using namespace std;
using namespace Eigen;

namespace tobas_fixed_wing_controller
{
FixedWingMicroDisturbanceDynamics::FixedWingMicroDisturbanceDynamics(const Drone& drone)
  : drone_(drone), kdl_model_(drone.tree())
{
  hor_prop_idxes_ = drone.rotorConfigIdxInAxis(Axis::X_POSITIVE);
  setInputLimits();
}

void FixedWingMicroDisturbanceDynamics::update(double V)
{
  assert(V > 0.);

  // TODO
}

const FixedWingMicroDisturbanceDynamics::StateVector&
FixedWingMicroDisturbanceDynamics::trimState() const
{
  return x_0_;
}

const VectorXd& FixedWingMicroDisturbanceDynamics::trimInput() const
{
  return u_0_;
}

const VectorXd& FixedWingMicroDisturbanceDynamics::minInput() const
{
  return min_u_;
}

const VectorXd& FixedWingMicroDisturbanceDynamics::maxInput() const
{
  return max_u_;
}

VectorXd FixedWingMicroDisturbanceDynamics::minDeltaInput() const
{
  return min_u_ - u_0_;
}

VectorXd FixedWingMicroDisturbanceDynamics::maxDeltaInput() const
{
  return max_u_ - u_0_;
}

double FixedWingMicroDisturbanceDynamics::trimState_u() const
{
  return x_0_(stateIdx_u);
}

double FixedWingMicroDisturbanceDynamics::trimState_alpha() const
{
  return x_0_(stateIdx_alpha);
}

double FixedWingMicroDisturbanceDynamics::trimState_beta() const
{
  return x_0_(stateIdx_beta);
}

double FixedWingMicroDisturbanceDynamics::trimState_phi() const
{
  return x_0_(stateIdx_phi);
}

double FixedWingMicroDisturbanceDynamics::trimState_theta() const
{
  return x_0_(stateIdx_theta);
}

double FixedWingMicroDisturbanceDynamics::trimState_p() const
{
  return x_0_(stateIdx_p);
}

double FixedWingMicroDisturbanceDynamics::trimState_q() const
{
  return x_0_(stateIdx_q);
}

double FixedWingMicroDisturbanceDynamics::trimState_r() const
{
  return x_0_(stateIdx_r);
}

uint32_t FixedWingMicroDisturbanceDynamics::horizontalPropIndex(uint32_t input_index) const
{
  return hor_prop_idxes_[input_index];
}

uint32_t FixedWingMicroDisturbanceDynamics::horizontalPropsSize() const
{
  return hor_prop_idxes_.size();
}

uint32_t FixedWingMicroDisturbanceDynamics::controlSurfacesSize() const
{
  return drone_.fixedWingConfig().control_surfaces.size();
}

uint32_t FixedWingMicroDisturbanceDynamics::inputSize() const
{
  return horizontalPropsSize() + controlSurfacesSize();
}

void FixedWingMicroDisturbanceDynamics::setInputLimits()
{
  for (int i = 0; i < horizontalPropsSize(); ++i)
  {
    min_u_(i) = 0.;
    max_u_(i) = drone_.maxThrust(horizontalPropIndex(i));
  }

  for (int i = 0; i < controlSurfacesSize(); ++i)
  {
    min_u_(horizontalPropsSize() + i) =
      drone_.fixedWingConfig().control_surfaces[i].angle_limit.lower;
    max_u_(horizontalPropsSize() + i) =
      drone_.fixedWingConfig().control_surfaces[i].angle_limit.upper;
  }
}
}  // namespace tobas_fixed_wing_controller
