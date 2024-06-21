#pragma once

#include <tobas_std_tools/unordered_set.hpp>

#include "./treesolveri.hpp"

namespace kdl
{
/* セグメントの位置姿勢に影響を与える関節を抽出する． */
class TreeActiveJointsExtractor : public TreeSolverI
{
  using super = TreeSolverI;

public:
  explicit TreeActiveJointsExtractor(const Tree& tree);

  void updateInternalDataStructures() override;

  int solve(const std::vector<std::string>& endpoints);

  inline const std::vector<std::string>& activeJointNames() const;
  inline bool isActiveJoint(const std::string& jnt_name) const;

private:
  std::vector<std::string> active_joints_vec_;
  std::unordered_set<std::string> active_joints_set_;
};

inline const std::vector<std::string>& TreeActiveJointsExtractor::activeJointNames() const
{
  return active_joints_vec_;
}

inline bool TreeActiveJointsExtractor::isActiveJoint(const std::string& jnt_name) const
{
  return tobas_std::contains(active_joints_set_, jnt_name);
}
}  // namespace kdl
