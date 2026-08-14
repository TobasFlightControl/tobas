// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <tobas_drone_core/drone.hpp>
#include <tobas_kdl/tree_inertia_solver.hpp>

#include "./solver_i.hpp"

namespace tobas
{
namespace fixed_wing
{
/**
 * @brief Convert the reference frame of aerodynamic stability derivatives
 * for moments from the aerodynamic center to the center of gravity.
 */
class StabilityDerivativesCG : public SolverI
{
public:
  explicit StabilityDerivativesCG(const Drone& drone, const kdl::Tree& tree);

  bool updateInternalDataStructures() override;

  int update(const kdl::JntArray& q);

  inline const double& cPitchAlpha() const;
  inline const double& cYawBeta() const;
  inline const double& cPitchDelta(const std::string& link_name) const;
  inline const double& cYawDelta(const std::string& link_name) const;

private:
  const Drone& drone_;
  const kdl::Tree& tree_;

  kdl::TreeInertiaSolver inertia_solver_;

  double c_pitch_alpha_cg_;
  double c_yaw_beta_cg_;
  std::map<std::string, double> c_pitch_delta_cg_;
  std::map<std::string, double> c_yaw_delta_cg_;

  void clear();
};

inline const double& StabilityDerivativesCG::cPitchAlpha() const
{
  return c_pitch_alpha_cg_;
}

inline const double& StabilityDerivativesCG::cYawBeta() const
{
  return c_yaw_beta_cg_;
}

inline const double& StabilityDerivativesCG::cPitchDelta(const std::string& link_name) const
{
  return c_pitch_delta_cg_.at(link_name);
}

inline const double& StabilityDerivativesCG::cYawDelta(const std::string& link_name) const
{
  return c_yaw_delta_cg_.at(link_name);
}
}  // namespace fixed_wing
}  // namespace tobas
