// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <chrono>

#include <tobas_kdl/frame_acc.hpp>
#include <tobas_kdl/trajectory.hpp>
#include <tobas_kdl/tree_fk_solver_pos.hpp>

namespace tobas
{
namespace lr_tools
{
/* Compute the swing-leg foot-tip state in the Cartesian coordinate system. */
class SwingLegController
{
  static constexpr double kDefaultRaibertGain = 0.03;
  static constexpr double kDefaultStandPeriod = 1.;
  static constexpr double kDefaultSwingPeriod = 1.;
  static constexpr double kDefaultClearance = 0.01;

  using TimeType = std::chrono::steady_clock::time_point;
  using DurationType = std::chrono::duration<double>;

public:
  explicit SwingLegController(
    const kdl::Tree& tree,
    const std::vector<std::string>& thigh_names,
    const std::vector<std::string>& foot_names);

  bool updateInternalDataStructures();

  void reset();

  bool update(
    double z,
    const kdl::Vector& G_Vel_GB,
    const kdl::Rotation& W_Rot_B,
    const kdl::Vector& G_Gyro_GB,
    const kdl::JntArray& q,
    const std::vector<bool>& is_stand,
    const TimeType& cur_time);

  bool setRaibertGain(double raibert_gain);
  bool setClearance(double clearance);
  bool setGaitParams(double stand_period, double swing_period);
  bool setVelocity(double vx, double vy, double yawrate);

  inline const kdl::VectorAcc& getFootState(size_t leg) const;

private:
  const kdl::Tree& tree_;
  const std::vector<std::string> thigh_names_, foot_names_;
  const size_t nc_;  // The number of contact points

  // Config
  double raibert_gain_ = kDefaultRaibertGain;  // [-]
  double clearance_ = kDefaultClearance;       // [m]
  double stand_period_ = kDefaultStandPeriod;  // [s]
  double swing_period_ = kDefaultSwingPeriod;  // [s]
  double vx_ = 0.;                             // [m/s]
  double vy_ = 0.;                             // [m/s]
  double yawrate_ = 0.;                        // [rad/s]

  kdl::TreeFkSolverPos fk_solver_;

  std::vector<kdl::CycloidGenerator3d>
    ref_traj_;                            // Generates the trajectory of {foot} relative to {gnd}, viewed from {gnd}.
  std::vector<bool> is_stand_prev_;       // Contact state of each foot.
  std::vector<TimeType> t_switch_;        // Time when each leg switched from stance to swing.
  std::vector<kdl::VectorAcc> B_Tdd_BF_;  // State of {foot} relative to {base}, viewed from {base}.
  std::vector<kdl::Vector> thigh_0_;      // Position of the leg root relative to {base}, viewed from {base}.
  kdl::VectorAcc G_Tdd_GF_;               // State of {foot} relative to {gnd}, viewed from {gnd}.
  double roll_, pitch_, yaw_;             // W_Rot_B

  void setThighOrigins();
};

inline const kdl::VectorAcc& SwingLegController::getFootState(size_t leg) const
{
  return B_Tdd_BF_[leg];
}
}  // namespace lr_tools
}  // namespace tobas
