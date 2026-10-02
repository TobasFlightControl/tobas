// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

namespace tobas
{
namespace kdl
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
}  // namespace kdl
}  // namespace tobas
