// SPDX-License-Identifier: Apache-2.0
// Copyright (C) 2026 Tobas, Inc.

#pragma once

namespace tobas
{
// Use as high a sampling rate as practical to prevent aliasing.
static constexpr int kImuSamplingRate = 800;  // [Hz]

static constexpr double kStaticAccThresh = 1.0;   // [m/s^2]
static constexpr double kStaticGyroThresh = 0.1;  // [rad/s]
}  // namespace tobas
