// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

namespace tobas
{
namespace dsp
{
/** Prewarping for bilinear transformation. */
double prewarp(double _wc, double _dt);

/** Convert the filter time constant [s] to the cutoff frequency [Hz]. */
double cutoffFromTimeConst(double _tau);
}  // namespace dsp
}  // namespace tobas
