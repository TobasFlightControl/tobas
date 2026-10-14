// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.
// Source: https://github.com/orocos/orocos_kinematics_dynamics/blob/HEAD/orocos_kdl/src/solveri.hpp
// Modified by Tobas, Inc.

// Copyright (C) 2013 Stephen Roderick <kiwi dot net at mac dot com>

// This library is free software; you can redistribute it and/or
// modify it under the terms of the GNU Lesser General Public
// License as published by the Free Software Foundation; either
// version 2.1 of the License, or (at your option) any later version.

// This library is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
// Lesser General Public License for more details.

// You should have received a copy of the GNU Lesser General Public
// License along with this library; if not, write to the Free Software
// Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA

#pragma once

namespace tobas::kdl
{
/**
 * Common structure-update interface for kinematics and dynamics solvers.
 * Solving methods check their requirements with assertions.
 * Dynamically allocated results are preallocated and returned by const reference.
 * These results are overwritten by the next solve and invalidated by structure updates
 * or destruction of the owning solver. Copy a result when it must outlive those operations.
 */
class SolverI
{
public:
  /**
   * Update the internal data structures. This is required if the number
   * of segments or number of joints of a chain/tree have changed.
   * Override this only when cached structure or workspaces need updating.
   */
  virtual void updateInternalDataStructures()
  {
  }
};
}  // namespace tobas::kdl
