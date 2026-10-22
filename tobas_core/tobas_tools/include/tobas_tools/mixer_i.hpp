// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <cmath>
#include <expected>
#include <map>
#include <string>

#include <tobas_drone_core/drone.hpp>
#include <tobas_kdl/tree.hpp>

namespace tobas
{
/** Base class for mixers. */
class MixerI
{
public:
  explicit MixerI(const Drone& _drone, const kdl::Tree& _tree);

  virtual void updateInternalDataStructures();

  std::expected<void, std::string> setRotorLiveliness(const std::string& _link_name, bool _alive);

  inline bool isInitialized() const;

protected:
  const Drone& drone_;
  const kdl::Tree& tree_;

  inline bool isRotorAlive(const std::string& _link_name) const;

  static inline double thrustDeadband(double _thrust);
  static inline Eigen::VectorXd thrustDeadband(const Eigen::VectorXd& _thrusts);

private:
  static constexpr double kZeroThrustThresh = 1e-2;  ///< [N]

  std::map<std::string, bool> rotor_alive_;
};

inline bool MixerI::isInitialized() const
{
  return rotor_alive_.size() > 0;
}

inline bool MixerI::isRotorAlive(const std::string& _link_name) const
{
  const auto it = rotor_alive_.find(_link_name);
  return it != rotor_alive_.end() && it->second;
}

inline double MixerI::thrustDeadband(double _thrust)
{
  return std::abs(_thrust) > kZeroThrustThresh ? _thrust : 0.0;
}

inline Eigen::VectorXd MixerI::thrustDeadband(const Eigen::VectorXd& _thrusts)
{
  return (_thrusts.array().abs() > kZeroThrustThresh).select(_thrusts, 0.0);
}
}  // namespace tobas
