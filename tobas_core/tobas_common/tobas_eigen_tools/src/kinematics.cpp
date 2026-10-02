// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_eigen_tools/kinematics.hpp"

#include <cassert>
#include <limits>

#include <tobas_math/core.hpp>

namespace tobas
{
namespace eigen
{
namespace
{
[[maybe_unused]] constexpr auto kEps = std::numeric_limits<double>::epsilon();
}  // namespace

Eigen::Matrix3d angvelFromEulerrateGlobal(double pitch, double yaw)
{
  const auto cos_pitch = std::cos(pitch);
  const auto sin_pitch = std::sin(pitch);
  const auto cos_yaw = std::cos(yaw);
  const auto sin_yaw = std::sin(yaw);

  Eigen::Matrix3d res;
  res(0, 0) = cos_pitch * cos_yaw;
  res(0, 1) = -sin_yaw;
  res(0, 2) = 0;
  res(1, 0) = cos_pitch * sin_yaw;
  res(1, 1) = cos_yaw;
  res(1, 2) = 0;
  res(2, 0) = -sin_pitch;
  res(2, 1) = 0;
  res(2, 2) = 1;

  return res;
}

Eigen::Vector3d angvelFromEulerrateGlobal(const Eigen::Vector3d& rpyd, double pitch, double yaw)
{
  return angvelFromEulerrateGlobal(pitch, yaw) * rpyd;
}

Eigen::Matrix3d angvelFromEulerrateLocal(double roll, double pitch)
{
  const auto cos_roll = std::cos(roll);
  const auto sin_roll = std::sin(roll);
  const auto cos_pitch = std::cos(pitch);
  const auto sin_pitch = std::sin(pitch);

  Eigen::Matrix3d res;
  res(0, 0) = 1;
  res(0, 1) = 0;
  res(0, 2) = -sin_pitch;
  res(1, 0) = 0;
  res(1, 1) = cos_roll;
  res(1, 2) = sin_roll * cos_pitch;
  res(2, 0) = 0;
  res(2, 1) = -sin_roll;
  res(2, 2) = cos_roll * cos_pitch;

  return res;
}

Eigen::Vector3d angvelFromEulerrateLocal(const Eigen::Vector3d& rpyd, double roll, double pitch)
{
  return angvelFromEulerrateLocal(roll, pitch) * rpyd;
}

Eigen::Matrix3d eulerrateFromAngvelGlobal(double pitch, double yaw)
{
  const auto cos_pitch = std::cos(pitch);
  const auto tan_pitch = std::tan(pitch);
  const auto cos_yaw = std::cos(yaw);
  const auto sin_yaw = std::sin(yaw);
  assert(cos_pitch > kEps);

  Eigen::Matrix3d res;
  res(0, 0) = cos_yaw / cos_pitch;
  res(0, 1) = sin_yaw / cos_pitch;
  res(0, 2) = 0;
  res(1, 0) = -sin_yaw;
  res(1, 1) = cos_yaw;
  res(1, 2) = 0;
  res(2, 0) = cos_yaw * tan_pitch;
  res(2, 1) = sin_yaw * tan_pitch;
  res(2, 2) = 1;

  return res;
}

Eigen::Vector3d eulerrateFromAngvelGlobal(const Eigen::Vector3d& angvel, double pitch, double yaw)
{
  return eulerrateFromAngvelGlobal(pitch, yaw) * angvel;
}

Eigen::Matrix3d eulerrateFromAngvelLocal(double roll, double pitch)
{
  const auto cos_roll = std::cos(roll);
  const auto sin_roll = std::sin(roll);
  const auto cos_pitch = std::cos(pitch);
  const auto tan_pitch = std::tan(pitch);
  assert(cos_pitch > kEps);

  Eigen::Matrix3d res;
  res(0, 0) = 1;
  res(0, 1) = sin_roll * tan_pitch;
  res(0, 2) = cos_roll * tan_pitch;
  res(1, 0) = 0;
  res(1, 1) = cos_roll;
  res(1, 2) = -sin_roll;
  res(2, 0) = 0;
  res(2, 1) = sin_roll / cos_pitch;
  res(2, 2) = cos_roll / cos_pitch;

  return res;
}

Eigen::Vector3d eulerrateFromAngvelLocal(const Eigen::Vector3d& angvel, double roll, double pitch)
{
  return eulerrateFromAngvelLocal(roll, pitch) * angvel;
}

Eigen::Vector3d
euleraccFromAngaccGlobal(const Eigen::Vector3d& angvel, const Eigen::Vector3d& angacc, double pitch, double yaw)
{
  const auto cp = std::cos(pitch);
  const auto tp = std::tan(pitch);
  const auto cy = std::cos(yaw);
  const auto sy = std::sin(yaw);

  const auto rpyd = eulerrateFromAngvelGlobal(angvel, pitch, yaw);

  Eigen::Vector3d rpydd;
  rpydd.x() = rpyd.x() * rpyd.y() * tp + (angacc.x() + angvel.y() * rpyd.z()) * cy / cp +
              (angacc.y() - angvel.x() * rpyd.z()) * sy / cp;
  rpydd.y() = (angacc.y() - angvel.x() * rpyd.z()) * cy - (angacc.x() + angvel.y() * rpyd.z()) * sy;
  rpydd.z() = angacc.z() + (angvel.x() + cy + angvel.y() * sy) * rpyd.y() / math::sqr(cp) +
              (angacc.x() + angvel.y() * rpyd.z()) * cy * tp + (angacc.y() - angvel.x() * rpyd.z()) * sy * tp;

  return rpydd;
}

Eigen::Vector3d angaccFromEuleraccLocal(
  double roll,
  double pitch,
  double droll,
  double dpitch,
  double dyaw,
  double ddroll,
  double ddpitch,
  double ddyaw)
{
  const auto cr = std::cos(roll);
  const auto sr = std::sin(roll);
  const auto cp = std::cos(pitch);
  const auto sp = std::sin(pitch);

  const auto sr_sp = sr * sp;
  const auto sr_cp = sr * cp;
  const auto cr_sp = cr * sp;
  const auto cr_cp = cr * cp;

  const auto droll_dpitch = droll * dpitch;
  const auto dpitch_dyaw = dpitch * dyaw;
  const auto dyaw_droll = dyaw * droll;

  Eigen::Vector3d dgyro;
  dgyro.x() = ddroll - ddyaw * sp - dpitch_dyaw * cp;
  dgyro.y() = ddpitch * cr - droll_dpitch * sr + ddyaw * sr_cp + dyaw_droll * cr_cp - dpitch_dyaw * sr_sp;
  dgyro.z() = -ddpitch * sr - droll_dpitch * cr + ddyaw * cr_cp - dyaw_droll * sr_cp - dpitch_dyaw * cr_sp;

  return dgyro;
}

Eigen::Vector3d
angaccFromEuleraccLocal(double roll, double pitch, const Eigen::Vector3d& drpy, const Eigen::Vector3d& ddrpy)
{
  return angaccFromEuleraccLocal(roll, pitch, drpy.x(), drpy.y(), drpy.z(), ddrpy.x(), ddrpy.y(), ddrpy.z());
}
}  // namespace eigen
}  // namespace tobas
