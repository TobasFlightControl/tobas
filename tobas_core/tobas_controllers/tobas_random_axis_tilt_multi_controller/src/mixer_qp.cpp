// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_random_axis_tilt_multi_controller/mixer_qp.hpp"

#include <cmath>
#include <ranges>

#include <tobas_constants/scale.hpp>
#include <tobas_eigen_tools/core.hpp>
#include <tobas_eigen_tools/geometry.hpp>
#include <tobas_eigen_tools/operators.hpp>
#include <tobas_math/core.hpp>
#include <tobas_std_tools/universal_constants.hpp>

using namespace std;
using namespace Eigen;

namespace tobas
{
namespace random_axis_tilt_multicopter
{
namespace
{
constexpr size_t kCircleApproximationSides = 16;

/** Block matrix for approximating an upper bound on the norm of a 2D vector using a regular polygon (memo: 3-58). */
const Eigen::Matrix<double, kCircleApproximationSides, 2>& polygonBlockMatrix()
{
  static const auto block = []
  {
    Eigen::Matrix<double, kCircleApproximationSides, 2> matrix;
    for (size_t i = 0; i < kCircleApproximationSides; ++i) {
      const auto theta = 2.0 * M_PI * i / kCircleApproximationSides;
      matrix(i, 0) = std::cos(theta);
      matrix(i, 1) = std::sin(theta);
    }
    return matrix;
  }();
  return block;
}
}  // namespace

QpMixer::QpMixer(const Drone& drone, const kdl::Tree& tree)
  : super(drone, tree), fk_solver_(tree), inertia_solver_(tree)
{
}

bool QpMixer::updateInternalDataStructures()
{
  if (!super::updateInternalDataStructures()) {
    return false;
  }

  fk_solver_.updateInternalDataStructures();
  inertia_solver_.updateInternalDataStructures();

  const auto nr = drone_.prop->numRotors();
  info_.resize(nr);
  state_.resize(nr);

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

  const auto var_size = nr * 2;

  qp_.resize(var_size, 0, nr * kCircleApproximationSides);
  qp_.setZero();
  qp_.x_scale.setOnes();
  qp_.problem.A = eigen::blockDiag(polygonBlockMatrix(), nr);

  R_.resize(var_size);
  E_.resize(Eigen::NoChange, var_size);
  x_.resize(var_size);

  return true;
}

bool QpMixer::solve(
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

    // Left-hand side of the EoM matrix equality.
    {
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

      const auto col = 2 * idx;
      E_.block<3, 2>(0, col) = B;
      E_.block<3, 2>(3, col) = D;
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

  // Weights for the EoM.
  const auto linear_scale = mass * kAccelScale;                // [N]
  const auto angular_scale = (I_B.trace() / 3) * kDGyroScale;  // [Nm]
  Q_.diagonal().head<3>().fill(cfg_.linear_weight / math::sqr(linear_scale));
  Q_.diagonal().tail<3>().fill(cfg_.angular_weight / math::sqr(angular_scale));

  // Weights for thrust.
  const auto thrust_scale = mass * st::kGravity / drone_.prop->numRotors();  // [N]
  const auto thrust_weight_base = cfg_.thrust_weight / math::sqr(thrust_scale);
  for (const auto& [idx, rotor_it] : std::views::enumerate(drone_.prop->rotors)) {
    const auto& rotor = rotor_it.second;
    const auto weight = thrust_weight_base * rotor->effortWeight();
    R_.diagonal().segment<2>(idx * 2).fill(weight);
  }

  // Objective function.
  qp_.problem.P = E_.transpose() * Q_ * E_ + R_;
  qp_.problem.q = -E_.transpose() * Q_ * f_;

  // Inequality constraints.
  const auto rate = std::cos(M_PI / kCircleApproximationSides);
  for (const auto& [idx, rotor_it] : std::views::enumerate(drone_.prop->rotors)) {
    const auto& rotor = rotor_it.second;
    const auto max_thrust = rotor_alive_[rotor->link_name] ? drone_.prop->maxThrust(rotor->link_name) * rate : 0.0;
    qp_.problem.b.segment<kCircleApproximationSides>(idx * kCircleApproximationSides).fill(max_thrust);
  }

  // Solve the QPP.
  if (!qp_.solve()) {
    cerr << "QP failed: " << qp_.errorMessage() << endl;
    return false;
  }
  x_ = qp_.solution();

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

double QpMixer::getThrust(size_t idx) const
{
  return thrustDeadband(x_.segment<2>(2 * idx).norm());
}

double QpMixer::getTiltAngle(size_t idx) const
{
  const auto tx = thrustDeadband(x_(2 * idx));
  const auto ty = thrustDeadband(x_(2 * idx + 1));
  return std::atan2(ty, tx);
}

bool QpMixer::setLinearWeight(double p)
{
  if (p <= 0.0) {
    cerr << "Linear weight must be positive." << endl;
    return false;
  }

  cfg_.linear_weight = p;
  return true;
}

bool QpMixer::setAngularWeight(double p)
{
  if (p <= 0.0) {
    cerr << "Angular weight must be positive." << endl;
    return false;
  }

  cfg_.angular_weight = p;
  return true;
}

bool QpMixer::setThrustWeight(double p)
{
  if (p <= 0.0) {
    cerr << "Thrust weight must be positive." << endl;
    return false;
  }

  cfg_.thrust_weight = p;
  return true;
}

bool QpMixer::setTiltAxisSingularDeclinationLB(double lb_rad)
{
  if (lb_rad < 0.0) {
    cerr << "The lower bind of singular tilt axis declination must be non-negative." << endl;
    return false;
  }

  cfg_.singular_declination_lb = lb_rad;
  return true;
}

bool QpMixer::setTiltAxisSingularDeclinationUB(double ub_rad)
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
