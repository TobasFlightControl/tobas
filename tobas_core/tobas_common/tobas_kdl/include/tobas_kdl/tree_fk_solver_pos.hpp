// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <string>

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

  Frame jntToCart(const JntArray& q, const std::string& seg_name);

private:
  Frame recursiveFk(const JntArray& q, const SegmentMap::const_iterator& seg_it);
};
}  // namespace kdl
}  // namespace tobas
