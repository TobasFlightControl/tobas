// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <tobas_msgs_adapter/odometry_with_covariance_stamped.hpp>

namespace tobas
{
void odometryFrdToFlu(const tobas_msgs::Odometry& src, tobas_msgs::Odometry& des);
void odometryFluToFrd(const tobas_msgs::Odometry& src, tobas_msgs::Odometry& des);
void odometryFrdToFlu(tobas_msgs::Odometry& arg);
void odometryFluToFrd(tobas_msgs::Odometry& arg);
}  // namespace tobas
