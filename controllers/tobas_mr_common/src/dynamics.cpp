#include "../include/tobas_mr_common/dynamics.hpp"

using namespace std;
using namespace Eigen;
using namespace KDL;

namespace tobas_mr_common
{
MultirotorDynamicsComponents::MultirotorDynamicsComponents(const tobas::Drone& drone)
  : drone_(drone),
    fk_solver_(drone.tree()),
    inertia_solver_(drone.tree()),
    z_rotors_(drone, tobas::Axis::Z_POSITIVE)
{
  updateInternalDataStructures();
}

void MultirotorDynamicsComponents::updateInternalDataStructures()
{
  fk_solver_.updateInternalDataStructures();
  inertia_solver_.updateInternalDataStructures();
  z_rotors_.updateInternalDataStructures();

  mass_ = inertia_solver_.JntToMass();
}

const double& MultirotorDynamicsComponents::mass() const
{
  return mass_;
}

double MultirotorDynamicsComponents::dragRotorSum(const vector<double>& rot_speeds) const
{
  assert(rot_speeds.size() == drone_.numRotors());

  double res = 0.;
  for (uint32_t i = 0; i < z_rotors_.count(); ++i)
  {
    const auto& rotor_idx = z_rotors_.rotorIdx(i);
    const auto& cd = z_rotors_.dragConstant(i);
    const auto& rot_speed = rot_speeds[rotor_idx];
    res += cd * abs(rot_speed);
  }
  return res;
}

double MultirotorDynamicsComponents::thrustSum(const vector<double>& rot_speeds)
{
  return z_rotors_.thrustSum(rot_speeds);
}
}  // namespace tobas_mr_common
