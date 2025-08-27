#pragma once

#include <tobas_control/state_spaces.hpp>
#include <tobas_kdl/tree_fk_solver_pos.hpp>
#include <tobas_kdl/tree_inertia_solver.hpp>

namespace lr_tools
{
/**
 * @brief The linear dynamics of legged robots. (memo: 2-69)
 * cf. Dynamic Locomotion in the MIT Cheetah 3 Through Convex Model-Predictive Control [Carlo+, 2018]
 */
class LinearDynamics : public ctrl::LinearDynamics
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
  static constexpr size_t kStateSize = kGravIdx + 1;

  static constexpr size_t kForceSizePerLeg = 3;   // fx, fy, fz
  static constexpr size_t kTorqueSizePerLeg = 1;  // tz
  static constexpr size_t kInputSizePerLeg = kForceSizePerLeg + kTorqueSizePerLeg;

  explicit LinearDynamics(const kdl::Tree& tree, const std::vector<std::string>& foot_names);

  bool updateInternalDataStructures();
  void update(const double& roll, const double& pitch, const kdl::JntArray& q, const std::vector<bool>& is_stand);

  inline size_t forceIndex(const size_t& leg) const;
  inline size_t torqueIndex(const size_t& leg) const;

private:
  const std::vector<std::string> foot_names_;
  const size_t nc_;  // The number of contact points

  kdl::TreeFkSolverPos fk_solver_;
  kdl::TreeInertiaSolver inertia_solver_;

  void updateA(const double& pitch);
  void updateB(const double& roll, const double& pitch, const kdl::JntArray& q, const std::vector<bool>& is_stand);
};

inline size_t LinearDynamics::forceIndex(const size_t& leg) const
{
  return kInputSizePerLeg * leg;
}

inline size_t LinearDynamics::torqueIndex(const size_t& leg) const
{
  return kInputSizePerLeg * leg + kForceSizePerLeg;
}
}  // namespace lr_tools
