// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include "../state_spaces.hpp"

namespace tobas::ctrl
{
/** Base class for converting continuous-time state equations to discrete-time state equations. */
class BaseC2D
{
public:
  virtual LinearDynamics convert(const LinearDynamics& _cont, double _dt) = 0;
};
}  // namespace tobas::ctrl
