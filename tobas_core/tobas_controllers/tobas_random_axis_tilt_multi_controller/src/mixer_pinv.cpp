// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_random_axis_tilt_multi_controller/mixer_pinv.hpp"

#include <ranges>

#include <tobas_eigen_tools/core.hpp>
#include <tobas_eigen_tools/geometry.hpp>
#include <tobas_eigen_tools/operators.hpp>
#include <tobas_std_tools/universal_constants.hpp>

using namespace std;
using namespace Eigen;

namespace tobas
{
namespace random_axis_tilt_multicopter
{
namespace
{
constexpr double kMinVerticalForcePerMass = 1.0;  // [m/s^2]
}  // namespace

PinvMixer::PinvMixer(const Drone& drone, const kdl::Tree& tree)
  : super(drone, tree), fk_solver_(tree), inertia_solver_(tree)
{
}

bool PinvMixer::updateInternalDataStructures()
{
  if (!super::updateInternalDataStructures()) {
    return false;
  }

  fk_solver_.updateInternalDataStructures();
  inertia_solver_.updateInternalDataStructures();

  const auto nr = drone_.prop->numRotors();
  info_.resize(nr);
  state_.resize(nr);
  E_.conservativeResize(NoChange, 2 * nr);
  x_.conservativeResize(2 * nr);

  for (const auto& [idx, rotor_it] : views::enumerate(drone_.prop->rotors)) {
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

  return true;
}

bool PinvMixer::solve(
  const kdl::JntArray& cur_q,
  const kdl::Rotation& cur_rot,
  const kdl::Vector& cur_gyro_B,
  const kdl::Vector& tar_acc_W,
  const kdl::Vector& tar_dgyro_B,
  const kdl::Vector& ext_force_W,
  const kdl::Vector& ext_torque_B)
{
  // Compute forward kinematics.
  if (fk_solver_.jntToCart(cur_q) < 0) {
    cerr << "Forward kinematics failed: " << fk_solver_.errorMessage() << endl;
    return false;
  }

  // Compute mass properties.
  if (inertia_solver_.jntToCart(cur_q) < 0) {
    cerr << "Inertia solver failed: " << inertia_solver_.errorMessage() << endl;
    return false;
  }
  const auto& inertia = inertia_solver_.getInertia();
  const auto& mass = inertia.getMass();
  const auto B_Pos_B2G = inertia.getCOG();
  const auto I_B = inertia.getRotationalInertiaCoG();

  for (const auto& [idx, rotor_it] : views::enumerate(drone_.prop->rotors)) {
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
    const auto& B_T_gpar = fk_solver_.getFrame(gpar_seg.name());

    if (info.is_tilt) {
      // Compute the deviation angle between the tilt axis and vertical direction.
      const auto tilt_axis_B = B_T_gpar.M * par_joint.axis();
      const auto tilt_axis_W = cur_rot * tilt_axis_B;
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

      const Matrix<double, 3, 2> B = B_T_gpar.M.data * info.A;
      const Matrix3d C = eigen::skew(B_Pos_G2P.data) - (d * cm) * Diagonal3d(1, 1, 1);
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
  auto eom_trans_right_W = mass * (tar_acc_W - grav_W) - ext_force_W;  // [N]
  // When the acceleration magnitude is small, such as during landing,
  // the rate of change in the tilt-angle solution becomes relatively large.
  // The mixer ignores delay in tilt-angle tracking, so large tilt-angle displacement should be avoided.
  // Therefore, at minimum, ensure that thrust is generated vertically upward.
  eom_trans_right_W.z(max(eom_trans_right_W.z(), mass * kMinVerticalForcePerMass));
  f_.head<3>() = cur_rot.inverse(eom_trans_right_W).data;

  // Right-hand side of the rotational EoM.
  const auto eom_rot_right_B = I_B * tar_dgyro_B + cur_gyro_B * (I_B * cur_gyro_B) - ext_torque_B;  // [Nm]
  f_.tail<3>() = eom_rot_right_B.data;

  // Least-squares solution of `Ex = f`; minimize the L2 norm of `x` when redundant degrees of freedom exist.
  // TODO: Consider constraints on the absolute thrust value; a convex optimization problem may work well.
  x_ = E_.jacobiSvd(ComputeThinU | ComputeThinV).solve(f_);

  // Fix to the minimum value because the thrust solution corresponding to the singular state has become zero.
  for (const auto& [idx, rotor_it] : views::enumerate(drone_.prop->rotors)) {
    const auto& rotor = rotor_it.second;
    const auto& state = state_[idx];
    if (state.is_singular) {
      x_(2 * idx) = drone_.prop->minThrust(rotor->link_name);
      x_(2 * idx + 1) = 0.0;
    }
  }

  return true;
}

double PinvMixer::getThrust(size_t idx) const
{
  return thrustDeadband(x_.segment<2>(2 * idx).norm());
}

double PinvMixer::getTiltAngle(size_t idx) const
{
  const auto tx = thrustDeadband(x_(2 * idx));
  const auto ty = thrustDeadband(x_(2 * idx + 1));
  return std::atan2(ty, tx);
}

bool PinvMixer::setTiltAxisSingularDeclinationLB(double lb_rad)
{
  if (lb_rad < 0.0) {
    cerr << "The lower bind of singular tilt axis declination must be non-negative." << endl;
    return false;
  }

  cfg_.singular_declination_lb = lb_rad;
  return true;
}

bool PinvMixer::setTiltAxisSingularDeclinationUB(double ub_rad)
{
  if (ub_rad < 0.0) {
    cerr << "The upper bind of singular tilt axis declination must be non-negative." << endl;
    return false;
  }

  cfg_.singular_declination_ub = ub_rad;
  return true;
}
}  // namespace random_axis_tilt_multicopter
}  // namespace tobas
