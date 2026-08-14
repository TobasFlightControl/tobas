// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <tobas_drone_core/drone.hpp>
#include <tobas_kdl/tree_inertia_solver.hpp>
#include <tobas_std_tools/range.hpp>

#include "./fw_stability_derivatives_cog.hpp"
#include "./solver_i.hpp"

namespace tobas
{
namespace fixed_wing
{
/**
 * @brief Compute the longitudinal trim state.
 */
class TrimConditions : public SolverI
{
public:
  explicit TrimConditions(const Drone& drone, const kdl::Tree& tree);

  bool updateInternalDataStructures() override;

  /**
   * @brief Update the internal state.
   *
   * @param V Magnitude of aircraft velocity relative to wind speed [m/s].
   * @param rho Air density [kg/m^3].
   * @param q Joint angles [rad].
   *
   * @return Error Error code
   */
  int update(double V, const double& rho, const kdl::JntArray& q);

  inline const StabilityDerivativesCG& stabilityDerivativesCG() const;

  /* Index of the control surface used for pitch trim. */
  inline const std::string& elevatorLinkName() const;

  /* Angle of attack [rad]. */
  inline const double& alpha() const;
  /* Pitch angle [rad]. */
  inline const double& theta() const;
  /* Elevator deflection angle [rad]. */
  inline const double& elevator() const;
  /* Lift coefficient [-]. */
  inline const double& c_L() const;
  /* Drag coefficient [-]. */
  inline const double& c_D() const;
  /* Thrust coefficient [-]. */
  inline const double& c_T() const;
  /* Speed in the X-axis direction [m/s]. */
  inline const double& u() const;

  inline double minimumSpeed(const double& rho) const;
  inline double maximumSpeed(const double& rho) const;

  /**
   * @brief Range of velocity magnitudes that avoid stall.
   * cf. Blue book, p.85, (2.9-47, 2.9-49)
   *
   * @param rho Air density [kg/m^3].
   * @return st::Range<double> Range of velocity magnitudes.
   */
  st::Range<double> speedLimit(const double& rho) const;

  /* Velocity at which enough lift to raise the aircraft is generated even when the angle of attack is zero. */
  double takeOffSpeed(const double& rho) const;

private:
  const Drone& drone_;
  const kdl::Tree& tree_;

  kdl::TreeInertiaSolver inertia_solver_;
  StabilityDerivativesCG asd_cog_;

  // Fixed values.
  double W_;                    // Aircraft weight [N].
  std::string elev_link_name_;  // Control surface used to balance pitch rotation.
  double a_, b_;                // Constant parts of (2.9-49).

  double alpha_;     // Angle of attack at trim [rad].
  double elevator_;  // Elevator deflection at trim [rad].
  double c_L_;       // Lift coefficient at trim [-].
  double c_D_;       // Drag coefficient at trim [-].
  double c_T_;       // Thrust coefficient at trim [-].
  double u_;         // Speed in the X-axis direction at trim [m/s].
};

inline const StabilityDerivativesCG& TrimConditions::stabilityDerivativesCG() const
{
  return asd_cog_;
}

inline const std::string& TrimConditions::elevatorLinkName() const
{
  return elev_link_name_;
}

inline const double& TrimConditions::alpha() const
{
  return alpha_;
}

inline const double& TrimConditions::theta() const
{
  return alpha_;  // `theta = alpha` from level flight.
}

inline const double& TrimConditions::elevator() const
{
  return elevator_;
}

inline const double& TrimConditions::c_L() const
{
  return c_L_;
}

inline const double& TrimConditions::c_D() const
{
  return c_D_;
}

inline const double& TrimConditions::c_T() const
{
  return c_T_;
}

inline const double& TrimConditions::u() const
{
  return u_;
}

inline double TrimConditions::minimumSpeed(const double& rho) const
{
  return speedLimit(rho).lower;
}

inline double TrimConditions::maximumSpeed(const double& rho) const
{
  return speedLimit(rho).upper;
}
}  // namespace fixed_wing
}  // namespace tobas
