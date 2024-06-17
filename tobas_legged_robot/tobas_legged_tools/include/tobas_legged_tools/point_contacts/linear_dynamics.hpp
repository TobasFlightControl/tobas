#pragma once

#include <boost/array.hpp>

#include <tobas_eigen_tools/geometry.hpp>
#include <tobas_linear_control/state_spaces.hpp>
#include <tobas_kdl/treefksolverpos.hpp>
#include <tobas_kdl/treejnttoinertiasolver.hpp>

namespace tobas_legged_tools
{
/**
 * @brief The linear dynamics of point-contact legged robots.\n
 * Dynamic Locomotion in the MIT Cheetah 3 Through Convex Model-Predictive Control [Carlo+, 2018]
 */
template <size_t N>
class PointContactsLinearDynamics : public ctrl::LinearDynamics
{
public:
  static constexpr size_t kRollIdx = 0;
  static constexpr size_t kPitchIdx = 1;
  static constexpr size_t kAltIdx = 2;
  static constexpr size_t kGyroXIdx = 3;
  static constexpr size_t kGyroYIdx = 4;
  static constexpr size_t kGyroZIdx = 5;
  static constexpr size_t kVelXIdx = 6;
  static constexpr size_t kVelYIdx = 7;
  static constexpr size_t kVelZIdx = 8;
  static constexpr size_t kGravIdx = 9;

  static constexpr size_t kStateSize = 10;
  static constexpr size_t kInputSize = 3 * N;

  explicit PointContactsLinearDynamics(const tobas_kdl::Tree& tree, const boost::array<std::string, N>& foot_names);

  void updateInternalDataStructures();

  void
  update(const double& roll, const double& pitch, const tobas_kdl::JntArray& q, const boost::array<bool, N>& is_stand);

private:
  tobas_kdl::TreeFkSolverPos fk_solver_;
  tobas_kdl::TreeJntToInertiaSolver inertia_solver_;

  boost::array<std::string, N> foot_names_;

  void updateA(const double& pitch);
  void
  updateB(const double& roll, const double& pitch, const tobas_kdl::JntArray& q, const boost::array<bool, N>& is_stand);
};

template <size_t N>
PointContactsLinearDynamics<N>::PointContactsLinearDynamics(
  const tobas_kdl::Tree& tree,
  const boost::array<std::string, N>& foot_names)
  : ctrl::LinearDynamics(kStateSize, kInputSize), fk_solver_(tree), inertia_solver_(tree), foot_names_(foot_names)
{
  setZero();

  // Fill constant parts
  A(kPitchIdx, kGyroYIdx) = 1;
  A(kAltIdx, kVelZIdx) = 1;
  A(kVelZIdx, kGravIdx) = -1;
}

template <size_t N>
void PointContactsLinearDynamics<N>::updateInternalDataStructures()
{
  fk_solver_.updateInternalDataStructures();
  inertia_solver_.updateInternalDataStructures();
}

template <size_t N>
void PointContactsLinearDynamics<N>::update(
  const double& roll,
  const double& pitch,
  const tobas_kdl::JntArray& q,
  const boost::array<bool, N>& is_stand)
{
  updateA(pitch);
  updateB(roll, pitch, q, is_stand);
}

template <size_t N>
void PointContactsLinearDynamics<N>::updateA(const double& pitch)
{
  A(kRollIdx, kGyroXIdx) = 1 / cos(pitch);
}

template <size_t N>
void PointContactsLinearDynamics<N>::updateB(
  const double& roll,
  const double& pitch,
  const tobas_kdl::JntArray& q,
  const boost::array<bool, N>& is_stand)
{
  // B: Base, G: CoG, F: Footprint, C: Contact

  if (inertia_solver_.JntToCart(q) < 0)
    throw std::runtime_error("Inertia solver failed: " + inertia_solver_.errorMessage());

  const auto& inertia = inertia_solver_.getInertia();
  const auto& mass = inertia.getMass();
  const auto B_Pos_BG = inertia.getCOG();
  const auto I = inertia.getRotationalInertiaCoG();

  const auto F_Rot_B = tobas_kdl::Rotation::RPY(roll, pitch, 0.);
  const auto B_Rot_F = F_Rot_B.inverse();
  const Eigen::Matrix3d R_I_inv = F_Rot_B.data * I.data.inverse();

  for (size_t l = 0; l < N; ++l)
  {
    if (is_stand[l])
    {
      if (fk_solver_.JntToCart(q, foot_names_[l]) < 0)
        throw std::runtime_error("FK solver failed: " + fk_solver_.errorMessage());

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
