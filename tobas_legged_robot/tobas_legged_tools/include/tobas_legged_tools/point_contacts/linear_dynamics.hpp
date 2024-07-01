#pragma once

#include <tobas_linear_control/state_spaces.hpp>
#include <tobas_kdl/treefksolverpos.hpp>
#include <tobas_kdl/treejnttoinertiasolver.hpp>

namespace lr_tools
{
/**
 * @brief The linear dynamics of point-contact legged robots.\n
 * Dynamic Locomotion in the MIT Cheetah 3 Through Convex Model-Predictive Control [Carlo+, 2018]
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

  explicit LinearDynamics(const kdl::Tree& tree, const std::vector<std::string>& foot_names);

  void updateInternalDataStructures();
  void update(const double& roll, const double& pitch, const kdl::JntArray& q, const std::vector<bool>& is_stand);

private:
  const std::vector<std::string> foot_names_;
  const size_t nc_;  // The number of contact points

  kdl::TreeFkSolverPos fk_solver_;
  kdl::TreeJntToInertiaSolver inertia_solver_;

  void updateA(const double& pitch);
  void updateB(const double& roll, const double& pitch, const kdl::JntArray& q, const std::vector<bool>& is_stand);
};
}  // namespace lr_tools
