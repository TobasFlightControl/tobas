// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <tobas_constants/rc_input.hpp>
#include <tobas_math/core.hpp>
#include <tobas_node/node.hpp>

#include <tobas_msgs_adapter/odometry_with_covariance_stamped.hpp>
#include <tobas_msgs_adapter/rc_input.hpp>

namespace tobas
{
namespace rc
{
class BaseController
{
public:
  explicit BaseController();

  virtual bool requirePosition() = 0;
  virtual bool requireVelocity() = 0;
  virtual bool requireAttitude() = 0;
  virtual bool requireHeading() = 0;

  virtual void initialize(BaseNode* node, FlightMode mode) = 0;
  virtual void reset(const builtin_interfaces::msg::Time& stamp, const tobas_msgs::Odometry& setpoint, bool landed) = 0;
  virtual void update(const tobas_msgs::RCInput& rcin, const tobas_msgs::Odometry& odom, bool landed) = 0;

protected:
  static constexpr int kExpoScale = 100;
  static constexpr double kDeadband = 0.02;  // Adjust so RCInput snaps to zero in the dead zone

  /* Return 0 when the value is inside the deadband. */
  inline double deadband(double x) const;

  /* Project an RCInput value into the range [lb, ub]. */
  inline double remap(double x, double lb, double ub) const;

  /* Return 0 when the RCInput value is inside the deadband; otherwise project it into [lb, ub]. */
  inline double remapDead(double x, double lb, double ub) const;

  /* expo -> remap */
  inline double expoRemap(double x, double exp, double lb, double ub) const;

  /* dead -> expo -> remap */
  inline double expoRemapDead(double x, double exp, double lb, double ub) const;

  /* Probably the same as Futaba's EXPO function: [-1, 1] -> [-1, 1] */
  static inline double expo(double x, double exp);

  /* Add a flight mode prefix to the text. */
  static std::string addMode(const std::string& text, FlightMode mode);
};

inline double BaseController::deadband(double x) const
{
  return std::abs(x) < kDeadband ? 0. : x;
}

inline double BaseController::remap(double x, double lb, double ub) const
{
  return math::remap(x, kRcInputMin, kRcInputMax, lb, ub);
}

inline double BaseController::expoRemap(double x, double exp, double lb, double ub) const
{
  return remap(expo(x, exp), lb, ub);
}

inline double BaseController::remapDead(double x, double lb, double ub) const
{
  return remap(deadband(x), lb, ub);
}

inline double BaseController::expoRemapDead(double x, double exp, double lb, double ub) const
{
  return expoRemap(deadband(x), exp, lb, ub);
}

inline double BaseController::expo(double x, double exp)
{
  assert(std::abs(x) < 1.);
  assert(std::abs(exp) < 1.);
  return (1. + exp) * x - exp * math::sign(x) * math::sqr(x);
}
}  // namespace rc
}  // namespace tobas
