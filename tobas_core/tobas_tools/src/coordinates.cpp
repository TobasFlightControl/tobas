// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_tools/coordinates.hpp"

#include <tobas_kdl/conversion/coordinates.hpp>

namespace tobas
{
void odometryFrdToFlu(const tobas_msgs::Odometry& src, tobas_msgs::Odometry& dst)
{
  kdl::frameFrdToFlu(src.frame, dst.frame);
  kdl::twistFrdToFlu(src.twist, dst.twist);
  kdl::accelFrdToFlu(src.accel, dst.accel);
}

void odometryFluToFrd(const tobas_msgs::Odometry& src, tobas_msgs::Odometry& dst)
{
  odometryFrdToFlu(src, dst);
}

void odometryFrdToFlu(tobas_msgs::Odometry& arg)
{
  odometryFrdToFlu(arg, arg);
}

void odometryFluToFrd(tobas_msgs::Odometry& arg)
{
  odometryFluToFrd(arg, arg);
}
}  // namespace tobas
