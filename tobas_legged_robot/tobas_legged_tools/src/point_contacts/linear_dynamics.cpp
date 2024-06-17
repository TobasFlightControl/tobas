#include <tobas_eigen_tools/geometry.hpp>

#include "../../include/tobas_legged_tools/point_contacts/linear_dynamics.hpp"

using namespace std;
using namespace Eigen;
using namespace tobas_kdl;

namespace tobas_legged_tools
{
PointContactsLinearDynamics::PointContactsLinearDynamics(const Tree& tree, const vector<string>& foot_names)
  : fk_solver_(tree), inertia_solver_(tree), foot_names_(foot_names), nc_(foot_names.size())
{
  resize(kGravIdx + 1, nc_ * 3);
  setZero();

  // Fill constant parts
  A(kPitchIdx, kGyroYIdx) = 1;
  A(kAltIdx, kVelZIdx) = 1;
  A(kVelZIdx, kGravIdx) = -1;
}

void PointContactsLinearDynamics::updateInternalDataStructures()
{
  fk_solver_.updateInternalDataStructures();
  inertia_solver_.updateInternalDataStructures();
}

void PointContactsLinearDynamics::update(
  const double& roll,
  const double& pitch,
  const JntArray& q,
  const vector<bool>& is_stand)
{
  assert(is_stand.size() == nc_);

  updateA(pitch);
  updateB(roll, pitch, q, is_stand);
}

void PointContactsLinearDynamics::updateA(const double& pitch)
{
  A(kRollIdx, kGyroXIdx) = 1 / cos(pitch);
}

void PointContactsLinearDynamics::updateB(
  const double& roll,
  const double& pitch,
  const JntArray& q,
  const vector<bool>& is_stand)
{
  // B: Base, G: CoG, F: Footprint, C: Contact

  if (inertia_solver_.JntToCart(q) < 0)
    throw runtime_error("Inertia solver failed: " + inertia_solver_.errorMessage());

  const auto& inertia = inertia_solver_.getInertia();
  const auto& mass = inertia.getMass();
  const auto B_Pos_BG = inertia.getCOG();
  const auto I = inertia.getRotationalInertiaCoG();

  const auto F_Rot_B = Rotation::RPY(roll, pitch, 0.);
  const auto B_Rot_F = F_Rot_B.inverse();
  const Matrix3d R_I_inv = F_Rot_B.data * I.data.inverse();

  for (size_t l = 0; l < nc_; ++l)
  {
    if (is_stand[l])
    {
      if (fk_solver_.JntToCart(q, foot_names_[l]) < 0)
        throw runtime_error("FK solver failed: " + fk_solver_.errorMessage());

      const auto& B_Pos_BC = fk_solver_.getFrame().p;
      const auto B_Pos_GC = B_Pos_BC - B_Pos_BG;
      B.block<3, 3>(3, 3 * l) = R_I_inv * eigen_tools::crossMat(B_Pos_GC.data) * B_Rot_F.data;
      B.block<3, 3>(6, 3 * l).diagonal().fill(1 / mass);
    }
    else
    {
      B.block<6, 3>(3, 3 * l).setZero();
    }
  }
}
}  // namespace tobas_legged_tools
