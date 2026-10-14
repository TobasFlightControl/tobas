// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.
// Source: https://github.com/orocos/orocos_kinematics_dynamics/blob/HEAD/orocos_kdl/src/chaindynparam.cpp
// Modified by Tobas, Inc.

// Copyright  (C)  2009  Dominick Vanthienen <dominick dot vanthienen at intermodalics dot eu>

// Version: 1.0
// Author: Dominick Vanthienen <dominick dot vanthienen at intermodalics dot eu>
// Author: Ruben Smits <ruben dot smits at intermodalics dot eu>
// Maintainer: Ruben Smits <ruben dot smits at intermodalics dot eu>
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

#include "tobas_kdl/tree_dyn_param.hpp"

namespace tobas::kdl
{
TreeDynParam::TreeDynParam(const Tree& _tree, const Vector& _grav)
  : super(_tree), rne_coriolis_(tree_, kdl::Vector::Zero()), rne_gravity_(tree_, _grav)
{
  resize();
}

void TreeDynParam::updateInternalDataStructures()
{
  rne_coriolis_.updateInternalDataStructures();
  rne_gravity_.updateInternalDataStructures();

  resize();
}

const JntArray& TreeDynParam::jntToCoriolis(const JntArray& _q, const JntArray& _qd)
{
  return rne_coriolis_.cartToJnt(_q, _qd, q_zero_);
}

const JntArray& TreeDynParam::jntToGravity(const JntArray& _q)
{
  return rne_gravity_.cartToJnt(_q, q_zero_, q_zero_);
}

void TreeDynParam::resize()
{
  q_zero_ = JntArray::Zero(tree_.getNrOfJoints());
}
}  // namespace tobas::kdl
