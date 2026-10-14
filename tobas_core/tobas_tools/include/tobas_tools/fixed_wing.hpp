// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <eigen3/Eigen/Core>

#include <tobas_constants/fixed_wing.hpp>
#include <tobas_math/core.hpp>

namespace tobas
{
/**
 * Compute the angle of attack (`alpha`).
 *
 * @param _u,w Aircraft velocity relative to wind in the FRD coordinate system [m/s].
 * @return double Angle of attack [rad].
 */
inline double angleOfAttack(double _u, double _w)
{
  return _u > kMinAirSpeedThresh ? std::atan(_w / _u) : 0;
}

/**
 * Compute the angle of attack (`alpha`).
 *
 * @param _linvel_b Aircraft velocity relative to wind in the FRD coordinate system [m/s].
 * @return double Angle of attack [rad].
 */
inline double angleOfAttack(const Eigen::Vector3d& _linvel_b)
{
  return angleOfAttack(_linvel_b.x(), _linvel_b.z());
}

/**
 * Compute the sideslip angle (`beta`).
 *
 * @param _u,v,w Aircraft velocity relative to wind in the FRD coordinate system [m/s].
 * @return double Sideslip angle [rad].
 */
inline double angleOfSideSlip(double _u, double _v, double _w)
{
  const auto V = std::hypot(_u, _v, _w);
  return V > kMinAirSpeedThresh ? std::asin(_v / V) : 0;
}

/**
 * Compute the sideslip angle (`beta`).
 *
 * @param _linvel_b Aircraft velocity relative to wind in the FRD coordinate system [m/s].
 * @return double Sideslip angle [rad].
 */
inline double angleOfSideSlip(const Eigen::Vector3d& _linvel_b)
{
  return angleOfSideSlip(_linvel_b.x(), _linvel_b.y(), _linvel_b.z());
}

/**
 * Compute dynamic pressure (`q_bar`).
 *
 * @param _rho Air density [kg/m^3].
 * @param _v Magnitude of aircraft velocity relative to wind [m/s].
 * @return double Dynamic pressure [Pa].
 */
inline double dynamicPressure(double _rho, double _v)
{
  assert(_rho > 0);
  assert(_v >= 0);
  return _rho * math::sqr(_v) / 2;
}
}  // namespace tobas
