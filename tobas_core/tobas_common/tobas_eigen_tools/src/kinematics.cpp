// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_eigen_tools/kinematics.hpp"

#include <cassert>
#include <limits>

#include <tobas_math/core.hpp>

namespace tobas::eigen
{
namespace
{
[[maybe_unused]] constexpr auto kEps = std::numeric_limits<double>::epsilon();
}  // namespace

Eigen::Matrix3d angvelFromEulerrateGlobal(double _pitch, double _yaw)
{
  const auto cos_pitch = std::cos(_pitch);
  const auto sin_pitch = std::sin(_pitch);
  const auto cos_yaw = std::cos(_yaw);
  const auto sin_yaw = std::sin(_yaw);

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

Eigen::Vector3d angvelFromEulerrateGlobal(const Eigen::Vector3d& _rpyd, double _pitch, double _yaw)
{
  return angvelFromEulerrateGlobal(_pitch, _yaw) * _rpyd;
}

Eigen::Matrix3d angvelFromEulerrateLocal(double _roll, double _pitch)
{
  const auto cos_roll = std::cos(_roll);
  const auto sin_roll = std::sin(_roll);
  const auto cos_pitch = std::cos(_pitch);
  const auto sin_pitch = std::sin(_pitch);

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

Eigen::Vector3d angvelFromEulerrateLocal(const Eigen::Vector3d& _rpyd, double _roll, double _pitch)
{
  return angvelFromEulerrateLocal(_roll, _pitch) * _rpyd;
}

Eigen::Matrix3d eulerrateFromAngvelGlobal(double _pitch, double _yaw)
{
  const auto cos_pitch = std::cos(_pitch);
  const auto tan_pitch = std::tan(_pitch);
  const auto cos_yaw = std::cos(_yaw);
  const auto sin_yaw = std::sin(_yaw);
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

Eigen::Vector3d eulerrateFromAngvelGlobal(const Eigen::Vector3d& _angvel, double _pitch, double _yaw)
{
  return eulerrateFromAngvelGlobal(_pitch, _yaw) * _angvel;
}

Eigen::Matrix3d eulerrateFromAngvelLocal(double _roll, double _pitch)
{
  const auto cos_roll = std::cos(_roll);
  const auto sin_roll = std::sin(_roll);
  const auto cos_pitch = std::cos(_pitch);
  const auto tan_pitch = std::tan(_pitch);
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

Eigen::Vector3d eulerrateFromAngvelLocal(const Eigen::Vector3d& _angvel, double _roll, double _pitch)
{
  return eulerrateFromAngvelLocal(_roll, _pitch) * _angvel;
}

Eigen::Vector3d
euleraccFromAngaccGlobal(const Eigen::Vector3d& _angvel, const Eigen::Vector3d& _angacc, double _pitch, double _yaw)
{
  const auto cp = std::cos(_pitch);
  const auto tp = std::tan(_pitch);
  const auto cy = std::cos(_yaw);
  const auto sy = std::sin(_yaw);

  const auto rpyd = eulerrateFromAngvelGlobal(_angvel, _pitch, _yaw);

  Eigen::Vector3d rpydd;
  rpydd.x() = rpyd.x() * rpyd.y() * tp + (_angacc.x() + _angvel.y() * rpyd.z()) * cy / cp +
              (_angacc.y() - _angvel.x() * rpyd.z()) * sy / cp;
  rpydd.y() = (_angacc.y() - _angvel.x() * rpyd.z()) * cy - (_angacc.x() + _angvel.y() * rpyd.z()) * sy;
  rpydd.z() = _angacc.z() + (_angvel.x() + cy + _angvel.y() * sy) * rpyd.y() / math::sqr(cp) +
              (_angacc.x() + _angvel.y() * rpyd.z()) * cy * tp + (_angacc.y() - _angvel.x() * rpyd.z()) * sy * tp;

  return rpydd;
}

Eigen::Vector3d angaccFromEuleraccLocal(
  double _roll,
  double _pitch,
  double _droll,
  double _dpitch,
  double _dyaw,
  double _ddroll,
  double _ddpitch,
  double _ddyaw)
{
  const auto cr = std::cos(_roll);
  const auto sr = std::sin(_roll);
  const auto cp = std::cos(_pitch);
  const auto sp = std::sin(_pitch);

  const auto sr_sp = sr * sp;
  const auto sr_cp = sr * cp;
  const auto cr_sp = cr * sp;
  const auto cr_cp = cr * cp;

  const auto droll_dpitch = _droll * _dpitch;
  const auto dpitch_dyaw = _dpitch * _dyaw;
  const auto dyaw_droll = _dyaw * _droll;

  Eigen::Vector3d dgyro;
  dgyro.x() = _ddroll - _ddyaw * sp - dpitch_dyaw * cp;
  dgyro.y() = _ddpitch * cr - droll_dpitch * sr + _ddyaw * sr_cp + dyaw_droll * cr_cp - dpitch_dyaw * sr_sp;
  dgyro.z() = -_ddpitch * sr - droll_dpitch * cr + _ddyaw * cr_cp - dyaw_droll * sr_cp - dpitch_dyaw * cr_sp;

  return dgyro;
}

Eigen::Vector3d
angaccFromEuleraccLocal(double _roll, double _pitch, const Eigen::Vector3d& _drpy, const Eigen::Vector3d& _ddrpy)
{
  return angaccFromEuleraccLocal(_roll, _pitch, _drpy.x(), _drpy.y(), _drpy.z(), _ddrpy.x(), _ddrpy.y(), _ddrpy.z());
}
}  // namespace tobas::eigen
