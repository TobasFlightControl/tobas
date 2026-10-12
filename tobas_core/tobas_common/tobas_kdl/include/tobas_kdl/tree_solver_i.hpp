// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include "./solver_i.hpp"
#include "./tree.hpp"

namespace tobas
{
namespace kdl
{
class TreeSolverI : public SolverI
{
public:
  inline explicit TreeSolverI(const Tree& tree);

protected:
  const Tree& tree_;
};

inline TreeSolverI::TreeSolverI(const Tree& tree) : tree_(tree)
{
}
}  // namespace kdl
}  // namespace tobas
