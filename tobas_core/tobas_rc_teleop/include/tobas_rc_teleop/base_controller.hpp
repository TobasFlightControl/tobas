// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <tobas_constants/rc_input.hpp>
#include <tobas_math/core.hpp>
#include <tobas_node/node.hpp>

#include <tobas_msgs_adapter/odometry_with_covariance_stamped.hpp>
#include <tobas_msgs_adapter/rc_input.hpp>

namespace tobas::rc
{
class BaseController
{
public:
  explicit BaseController();

  virtual bool requireHorizontalPosition() = 0;
  virtual bool requireVerticalPosition() = 0;
  virtual bool requireAttitude() = 0;
  virtual bool requireHeading() = 0;

  virtual void initialize(BaseNode* _node, FlightMode _mode) = 0;
  virtual void
  reset(const builtin_interfaces::msg::Time& _stamp, const tobas_msgs::Odometry& _setpoint, bool _landed) = 0;
  virtual void update(const tobas_msgs::RCInput& _rcin, const tobas_msgs::Odometry& _odom, bool _landed) = 0;

protected:
  static constexpr int kExpoScale = 100;
  static constexpr double kDeadband = 0.02;  ///< Adjust so RCInput snaps to zero in the dead zone

  /** Return 0 when the value is inside the deadband. */
  inline double deadband(double _x) const;

  /** Project an RCInput value into the range [lb, ub]. */
  inline double remap(double _x, double _lb, double _ub) const;

  /** Return 0 when the RCInput value is inside the deadband; otherwise project it into [lb, ub]. */
  inline double remapDead(double _x, double _lb, double _ub) const;

  /** expo -> remap */
  inline double expoRemap(double _x, double _exp, double _lb, double _ub) const;

  /** dead -> expo -> remap */
  inline double expoRemapDead(double _x, double _exp, double _lb, double _ub) const;

  /** Probably the same as Futaba's EXPO function: [-1, 1] -> [-1, 1] */
  static inline double expo(double _x, double _exp);

  /** Add a flight mode prefix to the text. */
  static std::string addMode(const std::string& _text, FlightMode _mode);
};

inline double BaseController::deadband(double _x) const
{
  return std::abs(_x) < kDeadband ? 0.0 : _x;
}

inline double BaseController::remap(double _x, double _lb, double _ub) const
{
  return math::remap(_x, kRcInputMin, kRcInputMax, _lb, _ub);
}

inline double BaseController::expoRemap(double _x, double _exp, double _lb, double _ub) const
{
  return remap(expo(_x, _exp), _lb, _ub);
}

inline double BaseController::remapDead(double _x, double _lb, double _ub) const
{
  return remap(deadband(_x), _lb, _ub);
}

inline double BaseController::expoRemapDead(double _x, double _exp, double _lb, double _ub) const
{
  return expoRemap(deadband(_x), _exp, _lb, _ub);
}

inline double BaseController::expo(double _x, double _exp)
{
  assert(std::abs(_x) < 1.0);
  assert(std::abs(_exp) < 1.0);
  return (1.0 + _exp) * _x - _exp * math::sign(_x) * math::sqr(_x);
}
}  // namespace tobas::rc
