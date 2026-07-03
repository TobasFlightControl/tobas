// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <eigen3/Eigen/Core>

namespace tobas
{
namespace eigen
{
/* Return the matrix that converts ZYX Euler angle rates to angular velocity expressed in the global frame. */
Eigen::Matrix3d angvelFromEulerrateGlobal(double pitch, double yaw);

/* Convert ZYX Euler angle rates to angular velocity expressed in the global frame. */
Eigen::Vector3d angvelFromEulerrateGlobal(const Eigen::Vector3d& eulerrate, double pitch, double yaw);

/* Return the matrix that converts ZYX Euler angle rates to angular velocity expressed in the local frame. */
Eigen::Matrix3d angvelFromEulerrateLocal(double roll, double pitch);

/* Convert ZYX Euler angle rates to angular velocity expressed in the local frame. */
Eigen::Vector3d angvelFromEulerrateLocal(const Eigen::Vector3d& eulerrate, double roll, double pitch);

/* Calculate the matrix that converts angular velocity expressed in the global frame to ZYX Euler angle rates. */
Eigen::Matrix3d eulerrateFromAngvelGlobal(double pitch, double yaw);

/* Convert angular velocity expressed in the global frame to ZYX Euler angle rates. */
Eigen::Vector3d eulerrateFromAngvelGlobal(const Eigen::Vector3d& angvel, double pitch, double yaw);

/**
 * @brief Calculate the matrix that converts angular velocity expressed in the local frame to ZYX Euler angle rates.
 * cf. https://www.sky-engin.jp/blog/eulerian-angles/#toc7
 */
Eigen::Matrix3d eulerrateFromAngvelLocal(double roll, double pitch);

/* Convert angular velocity expressed in the local frame to ZYX Euler angle rates. */
Eigen::Vector3d eulerrateFromAngvelLocal(const Eigen::Vector3d& angvel, double roll, double pitch);

/* Convert angular acceleration expressed in the global frame to Euler angle acceleration. */
Eigen::Vector3d
euleraccFromAngaccGlobal(const Eigen::Vector3d& angvel, const Eigen::Vector3d& angacc, double pitch, double yaw);

/* Convert Euler angle acceleration to angular acceleration expressed in the local frame (memo: 2-41). */
Eigen::Vector3d angaccFromEuleraccLocal(
  double roll,
  double pitch,
  double droll,
  double dpitch,
  double dyaw,
  double ddroll,
  double ddpitch,
  double ddyaw);

/* Convert Euler angle acceleration to angular acceleration expressed in the local frame (memo: 2-41). */
Eigen::Vector3d
angaccFromEuleraccLocal(double roll, double pitch, const Eigen::Vector3d& drpy, const Eigen::Vector3d& ddrpy);
}  // namespace eigen
}  // namespace tobas
