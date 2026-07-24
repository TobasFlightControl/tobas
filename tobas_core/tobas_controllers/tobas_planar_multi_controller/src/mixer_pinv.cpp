// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_planar_multi_controller/mixer_pinv.hpp"

#include <ranges>

#include <tobas_std_tools/universal_constants.hpp>

namespace tobas
{
namespace planar_multicopter
{
PinvMixer::PinvMixer(const Drone& drone, const kdl::Tree& tree)
  : super(drone, tree), fk_solver_(tree), inertia_solver_(tree)
{
}

bool PinvMixer::updateInternalDataStructures()
{
  if (!super::updateInternalDataStructures()) {
    return false;
  }

  if (!fk_solver_.updateInternalDataStructures()) {
    return false;
  }
  if (!inertia_solver_.updateInternalDataStructures()) {
    return false;
  }

  E_.conservativeResize(Eigen::NoChange, drone_.prop->numRotors());
  E_.bottomRows<1>().setOnes();  // Left-hand side of the thrust-sum equality.

  x_.conservativeResize(drone_.prop->numRotors());

  return true;
}

bool PinvMixer::solve(
  const kdl::JntArray& cur_q,
  const kdl::Vector& cur_gyro_B,
  const kdl::Vector& tar_dgyro_B,
  const double& tar_thrusts_sum,
  const kdl::Vector& ext_torque_B)
{
  assert(tar_thrusts_sum > 0);

  // Compute forward kinematics.
  if (fk_solver_.jntToCart(cur_q) < 0) {
    std::cerr << "Forward kinematics failed: " << fk_solver_.errorMessage() << std::endl;
    return false;
  }

  // Compute mass properties.
  if (inertia_solver_.jntToCart(cur_q) < 0) {
    std::cerr << "Inertia solver failed: " << inertia_solver_.errorMessage() << std::endl;
    return false;
  }
  const auto& inertia = inertia_solver_.getInertia();
  const auto B_Pos_B2G = inertia.getCOG();
  const auto I_B = inertia.getRotationalInertiaCoG();

  // Left-hand side of the EoM matrix equality.
  for (const auto& [idx, pair] : std::views::enumerate(drone_.prop->rotors)) {
    const auto& rotor = pair.second;

    if (rotor_alive_[rotor->link_name]) {
      const auto& B_Pos_B2P = fk_solver_.getFrame(rotor->link_name).p;

      const auto& elem = tree_.getSegment(rotor->link_name)->second;
      const auto& B_Rot_Par = fk_solver_.getFrame(elem.parent->first).M;
      const auto axis_B = B_Rot_Par * elem.segment.joint().axis();

      const auto d = rotor->sign();
      const auto cm = rotor->momentConst();
      const auto B_Pos_G2P = B_Pos_B2P - B_Pos_B2G;
      E_.block<3, 1>(0, idx) = (B_Pos_G2P * axis_B - (d * cm) * axis_B).data;
    }
    else {
      // When the rotor is dead, force the optimal thrust to zero by setting the transfer from thrust to expected motion to zero.
      E_.block<3, 1>(0, idx).setZero();
    }
  }

  // Right-hand side of the EoM matrix equality.
  f_.head<3>() = (I_B * tar_dgyro_B + cur_gyro_B * (I_B * cur_gyro_B) - ext_torque_B).data;  // [Nm]

  // Right-hand side of the thrust-sum equality.
  f_(3) = tar_thrusts_sum;

  // Solve `Ex = f`.
  // TODO: Assign per-row priorities (`atti > thrust > yaw`) when `Rank(E) < 4` and the equation cannot be solved.
  x_ = E_.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(f_);

  return true;
}

double PinvMixer::getThrust(size_t idx) const
{
  return thrustDeadband(x_(idx));
}
}  // namespace planar_multicopter
}  // namespace tobas
