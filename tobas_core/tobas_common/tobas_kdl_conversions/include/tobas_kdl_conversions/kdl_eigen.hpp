// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <eigen3/Eigen/Geometry>

#include <tobas_kdl/quaternion.hpp>

namespace tobas
{
namespace kdl
{
void quaternionKDLToEigen(const Quaternion& k, Eigen::Quaterniond& e);
void quaternionEigenToKDL(const Eigen::Quaterniond& e, Quaternion& k);
}  // namespace kdl
}  // namespace tobas
