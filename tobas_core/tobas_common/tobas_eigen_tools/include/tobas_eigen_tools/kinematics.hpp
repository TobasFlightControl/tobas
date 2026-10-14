// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <eigen3/Eigen/Core>

namespace tobas::eigen
{
/** Return the matrix that converts ZYX Euler angle rates to angular velocity expressed in the global frame. */
Eigen::Matrix3d angvelFromEulerrateGlobal(double _pitch, double _yaw);

/** Convert ZYX Euler angle rates to angular velocity expressed in the global frame. */
Eigen::Vector3d angvelFromEulerrateGlobal(const Eigen::Vector3d& _eulerrate, double _pitch, double _yaw);

/** Return the matrix that converts ZYX Euler angle rates to angular velocity expressed in the local frame. */
Eigen::Matrix3d angvelFromEulerrateLocal(double _roll, double _pitch);

/** Convert ZYX Euler angle rates to angular velocity expressed in the local frame. */
Eigen::Vector3d angvelFromEulerrateLocal(const Eigen::Vector3d& _eulerrate, double _roll, double _pitch);

/** Calculate the matrix that converts angular velocity expressed in the global frame to ZYX Euler angle rates. */
Eigen::Matrix3d eulerrateFromAngvelGlobal(double _pitch, double _yaw);

/** Convert angular velocity expressed in the global frame to ZYX Euler angle rates. */
Eigen::Vector3d eulerrateFromAngvelGlobal(const Eigen::Vector3d& _angvel, double _pitch, double _yaw);

/**
 * Calculate the matrix that converts angular velocity expressed in the local frame to ZYX Euler angle rates.
 *
 * @see https://www.sky-engin.jp/blog/eulerian-angles/#toc7
 */
Eigen::Matrix3d eulerrateFromAngvelLocal(double _roll, double _pitch);

/** Convert angular velocity expressed in the local frame to ZYX Euler angle rates. */
Eigen::Vector3d eulerrateFromAngvelLocal(const Eigen::Vector3d& _angvel, double _roll, double _pitch);

/** Convert angular acceleration expressed in the global frame to Euler angle acceleration. */
Eigen::Vector3d
euleraccFromAngaccGlobal(const Eigen::Vector3d& _angvel, const Eigen::Vector3d& _angacc, double _pitch, double _yaw);

/** Convert Euler angle acceleration to angular acceleration expressed in the local frame (memo: 2-41). */
Eigen::Vector3d angaccFromEuleraccLocal(
  double _roll,
  double _pitch,
  double _droll,
  double _dpitch,
  double _dyaw,
  double _ddroll,
  double _ddpitch,
  double _ddyaw);

/** Convert Euler angle acceleration to angular acceleration expressed in the local frame (memo: 2-41). */
Eigen::Vector3d
angaccFromEuleraccLocal(double _roll, double _pitch, const Eigen::Vector3d& _drpy, const Eigen::Vector3d& _ddrpy);
}  // namespace tobas::eigen
