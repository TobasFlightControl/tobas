// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <eigen3/Eigen/Geometry>

#include <tobas_kdl/quaternion.hpp>

namespace tobas
{
namespace kdl
{
void quaternionKDLToEigen(const Quaternion& _k, Eigen::Quaterniond& _e);
void quaternionEigenToKDL(const Eigen::Quaterniond& _e, Quaternion& _k);
}  // namespace kdl
}  // namespace tobas
