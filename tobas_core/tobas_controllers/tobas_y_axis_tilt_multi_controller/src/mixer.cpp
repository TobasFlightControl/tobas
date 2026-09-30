// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_y_axis_tilt_multi_controller/mixer.hpp"

#include <cassert>
#include <ranges>
#include <utility>

#include <tobas_eigen_tools/geometry.hpp>
#include <tobas_math/float.hpp>

namespace tobas
{
namespace y_axis_tilt_multicopter
{
Mixer::Mixer(const Drone& drone, const kdl::Tree& tree) : super(drone, tree), fk_solver_(tree), inertia_solver_(tree)
{
}

void Mixer::updateInternalDataStructures()
{
  super::updateInternalDataStructures();

  fk_solver_.updateInternalDataStructures();
  inertia_solver_.updateInternalDataStructures();

  // Compute forward kinematics.
  [[maybe_unused]] const auto fk_result = fk_solver_.jntToCart(kdl::JntArray::Zero(tree_.getNrOfJoints()));
  assert(fk_result >= 0);

  const auto nr = drone_.prop->numRotors();
  info_.resize(nr);
  state_.resize(nr);

  size_t col = 0;
  for (const auto& [idx, rotor_it] : std::views::enumerate(drone_.prop->rotors)) {
    const auto& rotor = rotor_it.second;
    auto& info = info_[idx];
    info.column = col;
    if (rotor->tilt_joint_name.empty()) {
      info.is_tilt = false;
      col += 1;
    }
    else {
      info.is_tilt = true;
      col += 2;
    }
  }

  E_.conservativeResize(Eigen::NoChange, col);

  for (const auto& [idx, rotor_it] : std::views::enumerate(drone_.prop->rotors)) {
    const auto& rotor = rotor_it.second;
    auto& info = info_[idx];

    if (info.is_tilt) {
      const auto& cur_elem = tree_.getSegment(rotor->link_name)->second;
      const auto& par_elem = cur_elem.parent->second;
      const auto& gpar_elem = par_elem.parent->second;

      // Store the sign of the tilt axis.
      const auto& B_T_gpar = fk_solver_.getFrame(gpar_elem.segment.name());
      const auto tilt_axis = B_T_gpar.M * par_elem.segment.joint().axis();  // Tilt axis viewed from the base link.
      const auto tilt_axis_y = tilt_axis.normalized().y();
      assert(math::isClose(std::abs(tilt_axis_y), 1.0));
      info.sign = math::sign(tilt_axis_y);
    }
  }
}

std::expected<MixerSolution, std::string> Mixer::solve(
  const kdl::JntArray& cur_q,
  const kdl::Vector& cur_gyro_B,
  const kdl::Vector& tar_dgyro_B,
  const double& ux,
  const double& uz,
  const kdl::Vector& ext_torque_B)
{
  // Compute forward kinematics.
  if (fk_solver_.jntToCart(cur_q) < 0) {
    return std::unexpected("Forward kinematics failed: " + fk_solver_.errorMessage());
  }

  // Compute mass properties.
  if (inertia_solver_.jntToCart(cur_q) < 0) {
    return std::unexpected("Inertia solver failed: " + inertia_solver_.errorMessage());
  }
  const auto& inertia = inertia_solver_.getInertia();
  const auto B_Pos_B2G = inertia.getCOG();
  const auto I_B = inertia.getRotationalInertiaCoG();

  for (const auto& [idx, rotor_it] : std::views::enumerate(drone_.prop->rotors)) {
    const auto& rotor = rotor_it.second;
    const auto& info = info_[idx];
    auto& state = state_[idx];

    const auto& cur_elem = tree_.getSegment(rotor->link_name)->second;
    const auto& cur_seg = cur_elem.segment;
    const auto& par_elem = cur_elem.parent->second;
    const auto& par_seg = par_elem.segment;

    const auto d_cm = rotor->sign() * rotor->momentConst();

    if (info.is_tilt) {
      // Update the rotor axis angle relative to the body frame at zero tilt angle.
      const auto& gpar_elem = par_elem.parent->second;
      const auto& gpar_seg = gpar_elem.segment;
      const auto& B_T_gpar = fk_solver_.getFrame(gpar_seg.name());
      const auto B_T_par = B_T_gpar * par_seg.pose(0.0);
      const auto n = B_T_par.M * cur_seg.joint().axis();
      state.alpha = std::atan2(n.x(), n.z());

      // Compute the left-hand side of the equations of motion.
      // When the rotor is dead, force the optimal thrust to zero by setting the transfer from thrust to vehicle motion to zero.
      if (rotor_alive_[rotor->link_name]) {
        // Compute the point of thrust application viewed from the grandparent link.
        const auto& par_joint = par_seg.joint();
        const auto gpar_T_cur = par_seg.frame() * cur_seg.frame();
        const auto gpar_P_gpar2P =
          eigen::projectPointOnToLine(par_joint.origin.data, par_joint.axis().data, gpar_T_cur.p.data);

        const auto B_Pos_B2P = B_T_gpar * gpar_P_gpar2P;
        const auto B_Pos_G2P = B_Pos_B2P - B_Pos_B2G;
        const auto col_tx = info.column;
        const auto col_tz = col_tx + 1;
        E_(0, col_tx) = -d_cm;
        E_(1, col_tx) = B_Pos_G2P.z();
        E_(2, col_tx) = -B_Pos_G2P.y();
        E_(0, col_tz) = B_Pos_G2P.y();
        E_(1, col_tz) = -B_Pos_G2P.x();
        E_(2, col_tz) = -d_cm;
        E_.block<2, 2>(3, col_tx).setIdentity();
      }
      else {
        E_.middleCols<2>(info.column).setZero();
      }
    }
    else {
      // Update the rotor axis angle relative to the body frame.
      const auto& B_T_par = fk_solver_.getFrame(par_seg.name());
      const auto n = B_T_par.M * cur_seg.joint().axis();
      state.alpha = std::atan2(n.x(), n.z());

      // Update the left-hand side of the equations of motion.
      if (rotor_alive_[rotor->link_name]) {
        const auto& B_Pos_B2P = fk_solver_.getFrame(cur_seg.name()).p;
        const auto B_Pos_G2P = B_Pos_B2P - B_Pos_B2G;
        E_.block<3, 1>(0, info.column) = (B_Pos_G2P * n - d_cm * n).data;
        E_(3, info.column) = std::sin(state.alpha);
        E_(4, info.column) = std::cos(state.alpha);
      }
      else {
        E_.col(info.column).setZero();
      }
    }
  }

  // Right-hand side of the equations of motion.
  const auto eom_rot_right_B = I_B * tar_dgyro_B + cur_gyro_B * (I_B * cur_gyro_B) - ext_torque_B;  // [Nm]
  f_.head<3>() = eom_rot_right_B.data;

  // Thrust-sum condition.
  f_(3) = ux;
  f_(4) = uz;

  // Least-squares solution of `Ex = f`; minimize the L2 norm of `x` when redundant degrees of freedom exist.
  // TODO: Consider constraints on the absolute thrust value; a convex optimization problem may work well.
  const auto x = E_.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(f_).eval();

  // Convert the solver variables to independently owned rotor commands.
  const auto nr = drone_.prop->numRotors();
  Eigen::VectorXd thrusts(nr);
  Eigen::VectorXd tilt_angles(nr);
  for (size_t idx = 0; idx < nr; ++idx) {
    const auto& info = info_[idx];
    if (info.is_tilt) {
      thrusts(idx) = thrustDeadband(x.segment<2>(info.column).norm());
      const auto tx = thrustDeadband(x(info.column));
      const auto tz = thrustDeadband(x(info.column + 1));
      tilt_angles(idx) = info.sign * (std::atan2(tx, tz) - state_[idx].alpha);
    }
    else {
      thrusts(idx) = thrustDeadband(x(info.column));
      tilt_angles(idx) = 0.0;
    }
  }

  return MixerSolution{ std::move(thrusts), std::move(tilt_angles) };
}
}  // namespace y_axis_tilt_multicopter
}  // namespace tobas
