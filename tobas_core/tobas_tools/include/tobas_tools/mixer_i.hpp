// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <tobas_drone_core/drone.hpp>
#include <tobas_kdl/tree.hpp>

namespace tobas
{
/**
 * @brief Base class for mixers.
 */
class MixerI
{
public:
  explicit MixerI(const Drone& drone, const kdl::Tree& tree);

  virtual bool updateInternalDataStructures();

  bool setRotorLiveliness(const std::string& link_name, bool alive);

  inline bool isInitialized() const;

protected:
  const Drone& drone_;
  const kdl::Tree& tree_;

  std::map<std::string, bool> rotor_alive_;

  /* Set tiny thrust values below the threshold to zero. */
  inline double thrustDeadband(double thrust) const;

private:
  bool is_initialized_ = false;
};

inline bool MixerI::isInitialized() const
{
  return is_initialized_;
}

inline double MixerI::thrustDeadband(double thrust) const
{
  constexpr double kZeroThrustThresh = 1e-2;  // [N]
  return std::abs(thrust) > kZeroThrustThresh ? thrust : 0.0;
}
}  // namespace tobas
