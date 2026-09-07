// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <tobas_nlp/newton_1d.hpp>

#include "./common/definitions.hpp"
#include "./filter/asymmetric_first_order_filter.hpp"
#include "./ice_rotor_model.hpp"

namespace tobas
{
namespace gazebo
{
class EngineModel
{
  using self = EngineModel;

public:
  explicit EngineModel(const IceRotorModelMap& rotors);

  bool initialize(const sdf::ElementConstPtr& sdf);

  /* Rotational speed [rad/s]. */
  double getSpeed() const;

  /* Rotational position [rad]. */
  double getPosition() const;

  /* Vibration force [N]. */
  double getVibrationForce();

  void setThrottle(const double& throttle);

  bool step(const double& dt);

private:
  const IceRotorModelMap& rotors_;

  // SDF parameters
  std::pair<double, double> engine_const_;  // A, B (memo: 3-28)
  double time_const_up_;                    // [s]
  double time_const_down_;                  // [s]
  double vibration_force_coef_;             // [N/(rad/s)^2]
  double vibration_force_variation_rate_;   // [-]
  double vibration_double_freq_coef_;       // [-]

  // Command
  double throttle_ = 0.0;  // Throttle opening [0, 1].

  // State
  double position_ = 0.0;  // Position [rad].
  AsymmetricFirstOrderFilter<double> speed_filter_;

  // Solver
  nlp::NewtonSolver1d newton_;

  // Random
  std::random_device rnd_dev_;
  std::mt19937 rnd_gen_;
  RiceDistribution rice_;

  bool getSdfParams(const sdf::ElementConstPtr& sdf);

  /* Compute steady-state rotational speed from engine throttle and propeller pitch angle (memo: 3-29). */
  double computeSteadySpeed();

  /* Function passed to the Newton-method solver (memo: 3-29). */
  double speedFunc(double omega) const;
  double speedFuncDeriv(double omega) const;

  double calc_phi() const;
  double calc_f() const;
  double calc_k() const;
};
}  // namespace gazebo
}  // namespace tobas
