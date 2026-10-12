// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_planar_multi_controller/mixer_pinv.hpp"

#include <ranges>

namespace tobas
{
namespace planar_multicopter
{
PinvMixer::PinvMixer(const Drone& _drone, const kdl::Tree& _tree)
  : super(_drone, _tree), fk_solver_(_tree), inertia_solver_(_tree)
{
}

void PinvMixer::updateInternalDataStructures()
{
  super::updateInternalDataStructures();

  fk_solver_.updateInternalDataStructures();
  inertia_solver_.updateInternalDataStructures();

  E_.conservativeResize(Eigen::NoChange, drone_.prop->numRotors());
  E_.bottomRows<1>().setOnes();  // Left-hand side of the thrust-sum equality.
}

std::expected<Eigen::VectorXd, std::string> PinvMixer::solve(
  const kdl::JntArray& _cur_q,
  const kdl::Vector& _cur_gyro_b,
  const kdl::Vector& _tar_dgyro_b,
  double _tar_thrusts_sum,
  const kdl::Vector& _ext_torque_b)
{
  assert(_tar_thrusts_sum > 0);

  // Compute forward kinematics.
  const auto& frames = fk_solver_.jntToCart(_cur_q);

  // Compute mass properties.
  const auto inertia = inertia_solver_.jntToCart(_cur_q);
  const auto B_Pos_B2G = inertia.getCOG();
  const auto I_B = inertia.getRotationalInertiaCoG();

  // Left-hand side of the EoM matrix equality.
  for (const auto& [idx, pair] : std::views::enumerate(drone_.prop->rotors)) {
    const auto& rotor = pair.second;

    if (rotor_alive_[rotor->link_name]) {
      const auto& B_Pos_B2P = frames.at(rotor->link_name).p;

      const auto& elem = tree_.getSegment(rotor->link_name)->second;
      const auto& B_Rot_Par = frames.at(elem.parent->first).M;
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
  f_.head<3>() = (I_B * _tar_dgyro_b + _cur_gyro_b * (I_B * _cur_gyro_b) - _ext_torque_b).data;  // [Nm]

  // Right-hand side of the thrust-sum equality.
  f_(3) = _tar_thrusts_sum;

  // Solve `Ex = f`.
  // TODO: Assign per-row priorities (`atti > thrust > yaw`) when `Rank(E) < 4` and the equation cannot be solved.
  const auto thrusts = E_.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(f_);
  return thrustDeadband(thrusts);
}

}  // namespace planar_multicopter
}  // namespace tobas
