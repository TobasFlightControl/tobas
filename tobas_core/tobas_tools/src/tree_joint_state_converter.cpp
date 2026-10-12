// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_tools/tree_joint_state_converter.hpp"

#include <exception>

namespace tobas
{
TreeJointStateConverter::TreeJointStateConverter(const kdl::Tree& tree) : super(tree), jnt_parser_(tree_)
{
  resize();
  setZero();
}

void TreeJointStateConverter::updateInternalDataStructures()
{
  jnt_parser_.updateInternalDataStructures();

  resize();
  setZero();
}

std::expected<void, std::string> TreeJointStateConverter::convert(const tobas_msgs::msg::JointStateArray& msg)
{
  for (const auto& state : msg.states) {
    try {
      const auto& kdl_idx = jnt_parser_.jointIndex(state.name);  // Index in the tree
      q_out_(kdl_idx) = state.position;
      qd_out_(kdl_idx) = state.velocity;
      f_out_(kdl_idx) = state.effort;
    }
    catch (const std::exception& e) {
      return std::unexpected(e.what());
    }
  }

  return {};
}

void TreeJointStateConverter::resize()
{
  q_out_.resize(tree_.getNrOfJoints());
  qd_out_.resize(tree_.getNrOfJoints());
  f_out_.resize(tree_.getNrOfJoints());
}

void TreeJointStateConverter::setZero()
{
  q_out_.setZero();
  qd_out_.setZero();
  f_out_.setZero();
}
}  // namespace tobas
