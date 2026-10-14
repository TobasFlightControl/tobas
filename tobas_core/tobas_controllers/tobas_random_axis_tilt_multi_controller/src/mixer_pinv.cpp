// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_random_axis_tilt_multi_controller/mixer_pinv.hpp"

#include <cassert>
#include <ranges>
#include <utility>

#include <tobas_eigen_tools/geometry.hpp>
#include <tobas_eigen_tools/operators.hpp>
#include <tobas_std_tools/universal_constants.hpp>

namespace tobas::random_axis_tilt_multicopter
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

  const auto nr = drone_.prop->numRotors();
  info_.resize(nr);
  state_.resize(nr);
  E_.conservativeResize(Eigen::NoChange, 2 * nr);

  for (const auto& [idx, rotor_it] : std::views::enumerate(drone_.prop->rotors)) {
    const auto& rotor = rotor_it.second;
    auto& info = info_[idx];

    info.is_tilt = !rotor->tilt_joint_name.empty();

    const auto& cur_elem = tree_.getSegment(rotor->link_name)->second;
    const auto& cur_seg = cur_elem.segment;
    const auto& cur_joint = cur_seg.joint();
    const auto& par_elem = cur_elem.parent->second;
    const auto& par_seg = par_elem.segment;
    const auto& par_joint = par_seg.joint();

    const auto& q = par_seg.frame().M * cur_joint.axis();  // Rotor axis viewed from the joint frame of the parent link.

    info.A.col(0) = q.data;
    if (info.is_tilt) {
      const auto& p = par_joint.axis();  // Tilt axis viewed from the grandparent link.
      info.A.col(1) = (p * q).data;
    }
    else {
      // For a non-tilt rotor, force the tilt angle to remain zero by setting the transfer from `sin(theta)` to zero.
      info.A.col(1).setZero();
    }
  }
}

std::expected<MixerSolution, std::string> PinvMixer::solve(
  const kdl::JntArray& _cur_q,
  const kdl::Rotation& _cur_rot,
  const kdl::Vector& _cur_gyro_b,
  const kdl::Vector& _tar_acc_w,
  const kdl::Vector& _tar_dgyro_b,
  const kdl::Vector& _ext_force_w,
  const kdl::Vector& _ext_torque_b)
{
  // Compute forward kinematics.
  const auto& frames = fk_solver_.jntToCart(_cur_q);

  // Compute mass properties.
  const auto inertia = inertia_solver_.jntToCart(_cur_q);
  const auto& mass = inertia.getMass();
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
    const auto& par_joint = par_seg.joint();
    const auto& gpar_elem = par_elem.parent->second;
    const auto& gpar_seg = gpar_elem.segment;

    // Get the grandparent frame.
    const auto& B_T_gpar = frames.at(gpar_seg.name());

    if (info.is_tilt) {
      // Compute the deviation angle between the tilt axis and vertical direction.
      const auto tilt_axis_B = B_T_gpar.M * par_joint.axis();
      const auto tilt_axis_W = _cur_rot * tilt_axis_B;
      auto declination = tilt_axis_W.argument(kdl::Vector::UnitZ());
      if (declination > M_PI_2) {
        declination = M_PI - declination;
      }

      // Update the singular state.
      if (state.is_singular) {
        if (declination > cfg_.singular_declination_ub) {
          state.is_singular = false;
        }
      }
      else {
        if (declination < cfg_.singular_declination_lb) {
          state.is_singular = true;
        }
      }
    }

    // Compute the left-hand side of the equations of motion.
    const auto col = 2 * idx;
    if (rotor_alive_[rotor->link_name] && !state.is_singular) {
      // Compute the point of thrust application viewed from the grandparent link.
      const auto gpar_T_cur = par_seg.frame() * cur_seg.frame();
      const auto gpar_P_gpar2P =
        eigen::projectPointOnToLine(par_joint.origin.data, par_joint.axis().data, gpar_T_cur.p.data);

      const auto B_Pos_B2P = B_T_gpar * gpar_P_gpar2P;
      const auto B_Pos_G2P = B_Pos_B2P - B_Pos_B2G;

      const auto d = rotor->sign();
      const auto cm = rotor->momentConst();

      const Eigen::Matrix<double, 3, 2> B = B_T_gpar.M.data * info.A;
      const Eigen::Matrix3d C = eigen::skew(B_Pos_G2P.data) - (d * cm) * Eigen::Diagonal3d(1, 1, 1);
      const auto D = C * B;

      E_.block<3, 2>(0, col) = B;
      E_.block<3, 2>(3, col) = D;
    }
    else {
      // When the state is singular or the rotor is dead,
      // force the optimal thrust to zero by setting the transfer from thrust to expected motion to zero.
      E_.middleCols<2>(col).setZero();
    }
  }

  // Right-hand side of the translational EoM.
  const kdl::Vector grav_W(0, 0, -st::kGravity);
  auto eom_trans_right_W = mass * (_tar_acc_w - grav_W) - _ext_force_w;  // [N]
  // When the acceleration magnitude is small, such as during landing,
  // the rate of change in the tilt-angle solution becomes relatively large.
  // The mixer ignores delay in tilt-angle tracking, so large tilt-angle displacement should be avoided.
  // Therefore, at minimum, ensure that thrust is generated vertically upward.
  constexpr double kMinVerticalForcePerMass = 1.0;  // [m/s^2]
  eom_trans_right_W.z(std::max(eom_trans_right_W.z(), mass * kMinVerticalForcePerMass));
  f_.head<3>() = _cur_rot.inverse(eom_trans_right_W).data;

  // Right-hand side of the rotational EoM.
  const auto eom_rot_right_B = I_B * _tar_dgyro_b + _cur_gyro_b * (I_B * _cur_gyro_b) - _ext_torque_b;  // [Nm]
  f_.tail<3>() = eom_rot_right_B.data;

  // Least-squares solution of `Ex = f`; minimize the L2 norm of `x` when redundant degrees of freedom exist.
  // TODO: Consider constraints on the absolute thrust value; a convex optimization problem may work well.
  Eigen::VectorXd x = E_.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(f_);

  // Fix to the minimum value because the thrust solution corresponding to the singular state has become zero.
  for (const auto& [idx, rotor_it] : std::views::enumerate(drone_.prop->rotors)) {
    const auto& rotor = rotor_it.second;
    const auto& state = state_[idx];
    if (state.is_singular) {
      const auto col = 2 * idx;
      x(col) = drone_.prop->minThrust(rotor->link_name);
      x(col + 1) = 0.0;
    }
  }

  // Convert the solver variables to independently owned rotor commands.
  const auto nr = drone_.prop->numRotors();
  Eigen::VectorXd thrusts(nr);
  Eigen::VectorXd tilt_angles(nr);
  for (size_t idx = 0; idx < nr; ++idx) {
    const auto col = 2 * idx;
    thrusts(idx) = thrustDeadband(x.segment<2>(col).norm());
    const auto tx = thrustDeadband(x(col));
    const auto ty = thrustDeadband(x(col + 1));
    tilt_angles(idx) = std::atan2(ty, tx);
  }

  return MixerSolution{ std::move(thrusts), std::move(tilt_angles) };
}

void PinvMixer::setTiltAxisSingularDeclinationLB(double _lb_rad)
{
  assert(_lb_rad >= 0.0);
  cfg_.singular_declination_lb = _lb_rad;
}

void PinvMixer::setTiltAxisSingularDeclinationUB(double _ub_rad)
{
  assert(_ub_rad >= 0.0);
  cfg_.singular_declination_ub = _ub_rad;
}
}  // namespace tobas::random_axis_tilt_multicopter
