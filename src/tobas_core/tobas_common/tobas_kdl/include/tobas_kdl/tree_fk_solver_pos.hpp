// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.
// Source: https://github.com/orocos/orocos_kinematics_dynamics/blob/HEAD/orocos_kdl/src/treefksolverpos_recursive.hpp
// Modified by Tobas, Inc.

// Copyright  (C)  2007  Ruben Smits <ruben dot smits at mech dot kuleuven dot be>
// Copyright  (C)  2008 Julia Jesse

// Version: 1.0
// Author: Ruben Smits <ruben dot smits at mech dot kuleuven dot be>
// Maintainer: Ruben Smits <ruben dot smits at mech dot kuleuven dot be>
// URL: http://www.orocos.org/kdl

// This library is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public
// License as published by the Free Software Foundation; either
// version 3 of the License, or (at your option) any later version.

// This library is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
// General Public License for more details.

// You should have received a copy of the GNU General Public
// License along with this library; if not, write to the Free Software
// Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA

#pragma once

#include <string>

#include "./frames.hpp"
#include "./jntarray.hpp"
#include "./tree_solver_i.hpp"

namespace tobas::kdl
{
class TreeFkSolverPos : public TreeSolverI
{
  using super = TreeSolverI;

public:
  explicit TreeFkSolverPos(const Tree& _tree);

  Frame jntToCart(const JntArray& _q, const std::string& _seg_name);

private:
  Frame recursiveFk(const JntArray& _q, const SegmentMap::const_iterator& _seg_it);
};
}  // namespace tobas::kdl
