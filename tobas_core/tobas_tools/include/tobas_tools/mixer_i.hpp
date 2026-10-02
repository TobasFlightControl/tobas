// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <cmath>
#include <expected>
#include <string>

#include <tobas_drone_core/drone.hpp>
#include <tobas_kdl/tree.hpp>

namespace tobas
{
/** Base class for mixers. */
class MixerI
{
public:
  explicit MixerI(const Drone& drone, const kdl::Tree& tree);

  virtual void updateInternalDataStructures();

  std::expected<void, std::string> setRotorLiveliness(const std::string& link_name, bool alive);

  inline bool isInitialized() const;

protected:
  const Drone& drone_;
  const kdl::Tree& tree_;

  std::map<std::string, bool> rotor_alive_;

  static inline double thrustDeadband(double thrust);
  static inline Eigen::VectorXd thrustDeadband(const Eigen::VectorXd& thrusts);

private:
  static constexpr double kZeroThrustThresh = 1e-2;  ///< [N]
};

inline bool MixerI::isInitialized() const
{
  return rotor_alive_.size() > 0;
}

inline double MixerI::thrustDeadband(double thrust)
{
  return std::abs(thrust) > kZeroThrustThresh ? thrust : 0.0;
}

inline Eigen::VectorXd MixerI::thrustDeadband(const Eigen::VectorXd& thrusts)
{
  return (thrusts.array().abs() > kZeroThrustThresh).select(thrusts, 0.0);
}
}  // namespace tobas
