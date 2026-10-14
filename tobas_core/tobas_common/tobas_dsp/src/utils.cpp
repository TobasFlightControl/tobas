// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_dsp/utils.hpp"

#include <cassert>
#include <cmath>

#include <tobas_math/definitions.hpp>

namespace tobas
{
namespace dsp
{
double prewarp(double _wc, double _dt)
{
  assert(_wc > 0.0);
  assert(_dt > 0.0);

  const auto dt_2 = _dt / 2.0;
  return std::tan(_wc * dt_2) / dt_2;
}

double cutoffFromTimeConst(double _tau)
{
  return 1.0 / (M_2PI * _tau);
}
}  // namespace dsp
}  // namespace tobas
