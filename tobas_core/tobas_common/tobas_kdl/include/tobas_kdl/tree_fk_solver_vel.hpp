// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include "./frame_vel.hpp"
#include "./jntarray.hpp"
#include "./tree_solver_i.hpp"

namespace tobas
{
namespace kdl
{
class TreeFkSolverVel : public TreeSolverI
{
  using super = TreeSolverI;

public:
  explicit TreeFkSolverVel(const Tree& tree);

  int jntToCart(const JntArray& q, const JntArray& qd, const std::string& seg_name);

  inline const FrameVel& getFrameVel() const;

private:
  FrameVel p_out_;

  FrameVel recursiveFk(const JntArray& q, const JntArray& qd, const SegmentMap::const_iterator& seg_it);
};

inline const FrameVel& TreeFkSolverVel::getFrameVel() const
{
  return p_out_;
}
}  // namespace kdl
}  // namespace tobas
