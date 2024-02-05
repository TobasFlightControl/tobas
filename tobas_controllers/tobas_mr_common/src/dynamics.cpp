#include <tobas_tools/constants.hpp>

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

  if (inertia_solver_.JntToCart(JntArray::Zero(drone_.tree().getNrOfJoints())) < 0)
    throw runtime_error("Inertia solver failed: " + inertia_solver_.errorMessage());
}

double MultirotorDynamicsComponents::dragRotorSum(const vector<double>& rot_speeds) const
{
  assert(rot_speeds.size() == drone_.numRotors());

  double res = 0.;
  for (size_t i = 0; i < z_rotors_.count(); ++i)
  {
    const auto& rotor_idx = z_rotors_.rotorIdx(i);
    const auto& cd = z_rotors_.dragConstant(i);
    const auto& rot_speed = rot_speeds[rotor_idx];
    res += cd * abs(rot_speed);
  }
  return res;
}

Vector MultirotorDynamicsComponents::relativePerpVel(
  const Rotation& rot,
  const Vector& vel_B,
  const Vector& wind_W)
{
  // TODO: 正確には機体フレームではなくプロペラの位置の速度を使う
  const auto relative_vel_B = vel_B - rot.inverse(wind_W);  // 風に対する相対速度
  return Vector(relative_vel_B.x(), relative_vel_B.y(), 0);
}

Vector MultirotorDynamicsComponents::horizontalForce(
  const Rotation& rot,
  const Vector& vel_B,
  const Vector& wind_W,
  const vector<double>& rot_speeds)
{
  assert(rot_speeds.size() == drone_.numRotors());

  const auto drag_rotor_sum = dragRotorSum(rot_speeds);
  const auto rel_vel_perp = relativePerpVel(rot, vel_B, wind_W);
  return -drag_rotor_sum * rel_vel_perp;
}

Vector MultirotorDynamicsComponents::horizontalMoment(
  const Rotation& rot,
  const Vector& vel_B,
  const Vector& wind_W,
  const JntArray& q,
  const vector<double>& rot_speeds)
{
  assert(rot_speeds.size() == drone_.numRotors());

  // 重心を求める
  if (inertia_solver_.JntToCart(q) < 0)
    throw runtime_error("Inertia solver failed: " + inertia_solver_.errorMessage());
  const auto P_base_cog = inertia_solver_.getInertia().getCOG();

  // 擬似的なモーメントアーム [Ns] を求める
  Vector h_momemt_arm = Vector::Zero();
  for (size_t i = 0; i < z_rotors_.count(); ++i)
  {
    // CoG -> Rotor の位置を求める
    if (fk_solver_.JntToCart(q, z_rotors_.linkName(i)) < 0)
      throw runtime_error("Forward kinematics failed: " + fk_solver_.errorMessage());
    const auto P_cog_rotor = fk_solver_.getFrame().p - P_base_cog;

    const auto& rotor_idx = z_rotors_.rotorIdx(i);
    const auto& cd = z_rotors_.dragConstant(i);
    const auto& rot_speed = rot_speeds[rotor_idx];
    h_momemt_arm += cd * abs(rot_speed) * P_cog_rotor;
  }

  const auto vel_perp = relativePerpVel(rot, vel_B, wind_W);
  return -h_momemt_arm * vel_perp;
}
}  // namespace tobas_mr_common
