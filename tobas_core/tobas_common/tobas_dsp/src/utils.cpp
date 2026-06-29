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
double prewarp(double wc, double dt)
{
  assert(wc > 0.);
  assert(dt > 0.);

  const auto dt_2 = dt / 2.;
  return std::tan(wc * dt_2) / dt_2;
}

double cutoffFromTimeConst(double tau)
{
  return 1. / (M_2PI * tau);
}
}  // namespace dsp
}  // namespace tobas
