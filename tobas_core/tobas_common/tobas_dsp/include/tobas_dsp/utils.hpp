// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

namespace tobas
{
namespace dsp
{
/* 双一次変換のプリワーピング． */
double prewarp(double wc, double dt);

/* Convert the filter time constant [s] to the cutoff frequency [Hz]. */
double cutoffFromTimeConst(double tau);
}  // namespace dsp
}  // namespace tobas
