// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_manipulation/util.hpp"

namespace tobas
{
namespace manipulation
{
std::vector<std::string> linkNames(const tobas_msgs::LinkStateArray& msg)
{
  std::vector<std::string> res;
  for (const auto& state : msg.states) {
    res.push_back(state.name);
  }
  return res;
}

std::expected<std::vector<std::string>, std::string>
findActiveJointNames(const kdl::Tree& tree, const std::vector<std::string>& endpoints)
{
  std::vector<std::string> active_joints;
  std::vector<bool> seen(tree.getNrOfJoints(), false);
  const auto root_it = tree.getRootSegment();

  for (const auto& seg_name : endpoints) {
    auto cur_it = tree.getSegment(seg_name);
    if (cur_it == tree.getSegments().end()) {
      return std::unexpected("The tree does not contain segment '" + seg_name + "'.");
    }

    while (cur_it != root_it) {
      const auto& cur_elem = cur_it->second;
      const auto& cur_joint = cur_elem.segment.joint();
      if (cur_joint.type != kdl::Joint::kFixed && !seen[cur_elem.q_nr]) {
        active_joints.push_back(cur_joint.name);
        seen[cur_elem.q_nr] = true;
      }
      cur_it = cur_elem.parent;
    }
  }

  return active_joints;
}
}  // namespace manipulation
}  // namespace tobas
