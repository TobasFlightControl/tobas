#include <dh_std_tools/standard_atmosphere.hpp>

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
  mass_ = kdl_model_.treeMass();

  resize(kStateSize, horizontalPropsSize() + controlSurfacesSize());

  x_0_ = StateVector::Zero();
  u_0_ = VectorXd::Zero(inputSize());
}

void FixedWingMicroDisturbanceDynamics::update(double V, double altitude)
{
  assert(V > 0.);
  assert(altitude > 0.);

  const double rho = dh_std::altitudeToDensity(altitude);

  updateTrimStateInput(V, rho);
  updateA(V, rho);
  updateB(V, rho);
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
  return x_0_(kStateIdx_u);
}

double FixedWingMicroDisturbanceDynamics::trimState_alpha() const
{
  return x_0_(kStateIdx_alpha);
}

double FixedWingMicroDisturbanceDynamics::trimState_beta() const
{
  return x_0_(kStateIdx_beta);
}

double FixedWingMicroDisturbanceDynamics::trimState_phi() const
{
  return x_0_(kStateIdx_phi);
}

double FixedWingMicroDisturbanceDynamics::trimState_theta() const
{
  return x_0_(kStateIdx_theta);
}

double FixedWingMicroDisturbanceDynamics::trimState_p() const
{
  return x_0_(kStateIdx_p);
}

double FixedWingMicroDisturbanceDynamics::trimState_q() const
{
  return x_0_(kStateIdx_q);
}

double FixedWingMicroDisturbanceDynamics::trimState_r() const
{
  return x_0_(kStateIdx_r);
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

void FixedWingMicroDisturbanceDynamics::updateTrimStateInput(double V, double rho)
{
  // TODO
}
void FixedWingMicroDisturbanceDynamics::updateA(double V, double rho)
{
  // TODO
}
void FixedWingMicroDisturbanceDynamics::updateB(double V, double rho)
{
  // TODO
}
}  // namespace tobas_fixed_wing_controller
