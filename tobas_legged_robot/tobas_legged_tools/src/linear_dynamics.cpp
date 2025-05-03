#include <tobas_eigen_tools/geometry.hpp>

#include "../include/tobas_legged_tools/linear_dynamics.hpp"

using namespace std;
using namespace Eigen;

namespace lr_tools
{
LinearDynamics::LinearDynamics(const kdl::Tree& tree, const vector<string>& foot_names)
  : foot_names_(foot_names), nc_(foot_names.size()), fk_solver_(tree), inertia_solver_(tree)
{
  resize(kStateSize, nc_ * kInputSizePerLeg);
  setZero();

  // Fill constant parts
  A(kPitchIdx, kGyroYIdx) = 1;
  A(kAltIdx, kVelZIdx) = 1;
  A(kVelZIdx, kGravIdx) = -1;
}

bool LinearDynamics::updateInternalDataStructures()
{
  if (!fk_solver_.updateInternalDataStructures()) {
    return false;
  }
  if (!inertia_solver_.updateInternalDataStructures()) {
    return false;
  }

  return true;
}

void LinearDynamics::update(const double& roll, const double& pitch, const kdl::JntArray& q, const vector<bool>& is_stand)
{
  assert(is_stand.size() == nc_);

  updateA(pitch);
  updateB(roll, pitch, q, is_stand);
}

void LinearDynamics::updateA(const double& pitch)
{
  A(kRollIdx, kGyroXIdx) = 1 / cos(pitch);
}

void LinearDynamics::updateB(const double& roll, const double& pitch, const kdl::JntArray& q, const vector<bool>& is_stand)
{
  // B: Base, G: CoG, F: Footprint, C: Contact

  if (inertia_solver_.JntToCart(q) < 0) {
    throw runtime_error("Inertia solver failed: " + inertia_solver_.errorMessage());
  }

  const auto& inertia = inertia_solver_.getInertia();
  const auto& mass = inertia.getMass();
  const auto B_Pos_BG = inertia.getCOG();
  const auto B_Ins = inertia.getRotationalInertiaCoG();

  const auto F_Rot_B = kdl::Rotation::RPY(roll, pitch, 0.);
  const auto B_Rot_F = F_Rot_B.inverse();

  const auto F_Ins = F_Rot_B * B_Ins;
  const Vector3d F_Ins_inv_z = F_Ins.data.inverse().col(2);
  const Matrix3d R_I_inv = F_Rot_B.data * B_Ins.data.inverse();

  for (size_t l = 0; l < nc_; ++l) {
    if (is_stand[l]) {
      if (fk_solver_.JntToCart(q, foot_names_[l]) < 0) {
        throw runtime_error("FK solver failed: " + fk_solver_.errorMessage());
      }

      const auto& B_Pos_BC = fk_solver_.getFrame().p;
      const auto B_Pos_GC = B_Pos_BC - B_Pos_BG;
      B.block<3, 3>(kGyroXIdx, forceIndex(l)) = R_I_inv * eigen::skew(B_Pos_GC.data) * B_Rot_F.data;
      B.block<3, 1>(kGyroXIdx, torqueIndex(l)) = F_Ins_inv_z;
      B.block<3, 3>(kVelXIdx, forceIndex(l)).diagonal().fill(1 / mass);
    }
    else {
      B.block<6, kInputSizePerLeg>(kGyroXIdx, forceIndex(l)).setZero();
    }
  }
}
}  // namespace lr_tools
