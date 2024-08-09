#include <tobas_constants/constants.hpp>

#include "../include/tobas_drone_tools/mr_dynamics.hpp"

using namespace std;
using namespace Eigen;

namespace tobas
{
MultirotorDynamicsComponents::MultirotorDynamicsComponents(const Drone& drone, const kdl::Tree& tree)
  : drone_(drone), tree_(tree), fk_solver_(tree), inertia_solver_(tree), z_rotors_(drone, Z_POSITIVE)
{
  updateInternalDataStructures();
}

void MultirotorDynamicsComponents::updateInternalDataStructures()
{
  fk_solver_.updateInternalDataStructures();
  inertia_solver_.updateInternalDataStructures();
  z_rotors_.updateInternalDataStructures();

  if (inertia_solver_.JntToCart(kdl::JntArray::Zero(tree_.getNrOfJoints())) < 0)
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

kdl::Vector MultirotorDynamicsComponents::relativePerpVel(
  const kdl::Rotation& rot,
  const kdl::Vector& vel_B,
  const kdl::Vector& wind_W)
{
  // TODO: 正確には機体フレームではなくプロペラの位置の速度を使う
  const auto relative_vel_B = vel_B - rot.inverse(wind_W);  // 風に対する相対速度
  return kdl::Vector(relative_vel_B.x(), relative_vel_B.y(), 0);
}

kdl::Vector MultirotorDynamicsComponents::horizontalForce(
  const kdl::Rotation& rot,
  const kdl::Vector& vel_B,
  const kdl::Vector& wind_W,
  const vector<double>& rot_speeds)
{
  assert(rot_speeds.size() == drone_.numRotors());

  const auto drag_rotor_sum = dragRotorSum(rot_speeds);
  const auto rel_vel_perp = relativePerpVel(rot, vel_B, wind_W);
  return -drag_rotor_sum * rel_vel_perp;
}

kdl::Vector MultirotorDynamicsComponents::horizontalMoment(
  const kdl::Rotation& rot,
  const kdl::Vector& vel_B,
  const kdl::Vector& wind_W,
  const kdl::JntArray& q,
  const vector<double>& rot_speeds)
{
  assert(rot_speeds.size() == drone_.numRotors());

  // 重心を求める
  if (inertia_solver_.JntToCart(q) < 0)
    throw runtime_error("Inertia solver failed: " + inertia_solver_.errorMessage());
  const auto P_base_cog = inertia_solver_.getInertia().getCOG();

  // 擬似的なモーメントアーム [Ns] を求める
  kdl::Vector h_momemt_arm = kdl::Vector::Zero();
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
}  // namespace tobas
