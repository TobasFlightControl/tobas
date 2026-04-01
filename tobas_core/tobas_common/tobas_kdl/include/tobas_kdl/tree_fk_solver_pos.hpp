// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include "./frames.hpp"
#include "./jntarray.hpp"
#include "./tree_solver_i.hpp"

namespace tobas
{
namespace kdl
{
class TreeFkSolverPos : public TreeSolverI
{
  using super = TreeSolverI;

public:
  explicit TreeFkSolverPos(const Tree& tree);

  int jntToCart(const JntArray& q, const std::string& seg_name);

  inline const Frame& getFrame() const;

private:
  Frame p_out_;

  Frame recursiveFk(const JntArray& q, const SegmentMap::const_iterator& seg_it);
};

inline const Frame& TreeFkSolverPos::getFrame() const
{
  return p_out_;
}
}  // namespace kdl
}  // namespace tobas
