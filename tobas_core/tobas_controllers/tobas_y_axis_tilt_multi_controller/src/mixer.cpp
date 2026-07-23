// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_y_axis_tilt_multi_controller/mixer.hpp"

#include <ranges>

#include <tobas_eigen_tools/core.hpp>
#include <tobas_eigen_tools/geometry.hpp>
#include <tobas_eigen_tools/operators.hpp>
#include <tobas_math/float.hpp>
#include <tobas_std_tools/universal_constants.hpp>

namespace tobas
{
namespace y_axis_tilt_multicopter
{
Mixer::Mixer(const Drone& drone, const kdl::Tree& tree) : super(drone, tree), fk_solver_(tree), inertia_solver_(tree)
{
}

bool Mixer::updateInternalDataStructures()
{
  if (!super::updateInternalDataStructures()) {
    return false;
  }

  if (!fk_solver_.updateInternalDataStructures()) {
    std::cerr << fk_solver_.errorMessage() << std::endl;
    return false;
  }
  if (!inertia_solver_.updateInternalDataStructures()) {
    std::cerr << inertia_solver_.errorMessage() << std::endl;
    return false;
  }

  // Compute forward kinematics.
  if (fk_solver_.jntToCart(kdl::JntArray::Zero(tree_.getNrOfJoints())) < 0) {
    std::cerr << fk_solver_.errorMessage() << std::endl;
    return false;
  }

  const auto nr = drone_.prop->numRotors();

  A_.conservativeResize(Eigen::NoChange, 2 * nr);
  for (size_t i = 0; i < nr; ++i) {
    A_.block<2, 2>(3, 2 * i).setIdentity();
  }
  x_.conservativeResize(2 * nr);

  thrust_points_.resize(nr);
  sign_.resize(nr);
  alpha_.resize(nr);

  for (const auto& [idx, rotor_it] : std::views::enumerate(drone_.prop->rotors)) {
    const auto& rotor = rotor_it.second;

    if (rotor->tilt_joint_name.empty()) {
      std::cerr << "The tilt joint of rotor " << rotor->link_name << " is not specified." << std::endl;
      return false;
    }

    const auto& cur_elem = tree_.getSegment(rotor->link_name)->second;
    const auto& cur_seg = cur_elem.segment;
    const auto& par_elem = cur_elem.parent->second;
    const auto& par_seg = par_elem.segment;
    const auto& par_joint = par_seg.joint();
    const auto& gpar_elem = par_elem.parent->second;
    const auto& gpar_seg = gpar_elem.segment;

    // Store the point of thrust application viewed from the grandparent link.
    const auto gpar_T_cur = par_seg.frame() * cur_seg.frame();
    const auto& rotor_pos = gpar_T_cur.p;
    const auto thrust_pos = eigen::projectPointOnToLine(par_joint.origin.data, par_joint.axis().data, rotor_pos.data);
    thrust_points_.at(idx) = thrust_pos;

    // Store the sign of the tilt axis.
    const auto& B_T_gpar = fk_solver_.getFrame(gpar_seg.name());
    const auto tilt_axis = B_T_gpar.M * par_joint.axis();  // Tilt axis viewed from the base link.
    const auto tilt_axis_y = tilt_axis.normalized().y();
    if (!math::isClose(std::abs(tilt_axis_y), 1.0)) {
      std::cerr << "Tilt axis must be parallel to the Y axis." << std::endl;
      return false;
    }
    sign_.at(idx) = math::sign(tilt_axis_y);
  }

  return true;
}

bool Mixer::solve(
  const kdl::JntArray& cur_q,
  const kdl::Vector& cur_gyro_B,
  const kdl::Vector& tar_dgyro_B,
  const double& ux,
  const double& uz,
  const kdl::Vector& ext_torque_B)
{
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

  for (const auto& [idx, rotor_it] : std::views::enumerate(drone_.prop->rotors)) {
    const auto& rotor = rotor_it.second;

    const auto& cur_elem = tree_.getSegment(rotor->link_name)->second;
    const auto& par_elem = cur_elem.parent->second;
    const auto& gpar_elem = par_elem.parent->second;

    // Store the offset of the tilt angle from vertically upward when the tilt-joint angle is zero.
    const auto& B_T_par = fk_solver_.getFrame(par_elem.segment.name());
    const auto n = B_T_par.M * cur_elem.segment.joint().axis();  // Rotation axis viewed from the base link.
    assert(math::isClose(n.y(), 0.0));  // Thrust is always generated only in the XZ plane of the body frame.
    alpha_.at(idx) = std::atan2(n.x(), n.z());

    // Compute the left-hand side of the equations of motion.
    const auto col_tx = 2 * idx;
    const auto col_tz = col_tx + 1;
    if (rotor_alive_.at(rotor->link_name)) {
      const auto& B_T_gpar = fk_solver_.getFrame(gpar_elem.segment.name());
      const auto B_Pos_B2P = B_T_gpar * thrust_points_.at(idx);
      const auto B_Pos_G2P = B_Pos_B2P - B_Pos_B2G;
      const auto d_cm = rotor->sign() * rotor->momentConst();
      A_(0, col_tx) = -d_cm;
      A_(1, col_tx) = B_Pos_G2P.z();
      A_(2, col_tx) = -B_Pos_G2P.y();
      A_(0, col_tz) = B_Pos_G2P.y();
      A_(1, col_tz) = -B_Pos_G2P.x();
      A_(2, col_tz) = -d_cm;
    }
    else {
      // When the rotor is dead, force the optimal thrust to zero by setting the transfer from thrust to vehicle motion to zero.
      A_.middleCols<2>(col_tx).setZero();
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
  x_ = A_.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(f_);

  return true;
}

double Mixer::getThrust(size_t idx) const
{
  return thrustDeadband(x_.segment<2>(2 * idx).norm());
}

double Mixer::getTiltAngle(size_t idx) const
{
  const auto tx = thrustDeadband(x_(2 * idx));
  const auto tz = thrustDeadband(x_(2 * idx + 1));
  return sign_.at(idx) * std::atan2(tx, tz) - alpha_.at(idx);
}
}  // namespace y_axis_tilt_multicopter
}  // namespace tobas
