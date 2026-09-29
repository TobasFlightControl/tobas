// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_random_axis_tilt_multi_controller/mixer_sqp.hpp"

#include <ranges>
#include <utility>

#include <tobas_constants/scale.hpp>
#include <tobas_eigen_tools/geometry.hpp>
#include <tobas_eigen_tools/operators.hpp>
#include <tobas_math/core.hpp>
#include <tobas_std_tools/universal_constants.hpp>

namespace tobas
{
namespace random_axis_tilt_multicopter
{
SqpMixer::SqpMixer(const Drone& drone, const kdl::Tree& tree)
  : super(drone, tree), joint_parser_(tree), fk_solver_(tree), inertia_solver_(tree), np_mixer_(drone, tree)
{
}

std::expected<void, std::string> SqpMixer::updateInternalDataStructures()
{
  if (const auto result = super::updateInternalDataStructures(); !result) {
    return result;
  }

  joint_parser_.updateInternalDataStructures();
  fk_solver_.updateInternalDataStructures();
  inertia_solver_.updateInternalDataStructures();
  if (const auto result = np_mixer_.updateInternalDataStructures(); !result) {
    return result;
  }

  resetTensors();

  return initializeSQP();
}

std::expected<MixerSolution, std::string> SqpMixer::solve(
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
    return std::unexpected("Forward kinematics failed: " + fk_solver_.errorMessage());
  }

  // Compute mass properties.
  if (inertia_solver_.jntToCart(cur_q) < 0) {
    return std::unexpected("Inertia solver failed: " + inertia_solver_.errorMessage());
  }
  const auto& inertia = inertia_solver_.getInertia();
  const auto& mass = inertia.getMass();
  const auto B_Pos_B2G = inertia.getCOG();
  const auto I_B = inertia.getRotationalInertiaCoG();

  for (const auto& [idx, rotor_it] : std::views::enumerate(drone_.prop->rotors)) {
    const auto& rotor = rotor_it.second;

    // Update B.
    const auto d = rotor->sign();
    const auto cm = rotor->momentConst();
    const auto& B_Pos_B2P = fk_solver_.getFrame(rotor->link_name).p;
    const auto r = B_Pos_B2P - B_Pos_B2G;
    B_.block<3, 3>(3, 3 * idx) = eigen::skew(r.data) - (d * cm) * Eigen::Diagonal3d(1, 1, 1);

    // Update ci0.
    const auto nr = drone_.prop->numRotors();
    if (rotor_alive_[rotor->link_name]) {
      if (!rotor->tilt_joint_name.empty()) {
        ci0_(idx) = joint_parser_.lowerLimit(rotor->tilt_joint_name);
        ci0_(nr + idx) = -joint_parser_.upperLimit(rotor->tilt_joint_name);
      }
      ci0_(2 * nr + idx) = drone_.prop->minThrust(rotor->link_name);
      ci0_(3 * nr + idx) = -drone_.prop->maxThrust(rotor->link_name);
    }
    else {
      if (!rotor->tilt_joint_name.empty()) {
        ci0_(idx) = 0.0;
        ci0_(nr + idx) = 0.0;
      }
      ci0_(2 * nr + idx) = 0.0;
      ci0_(3 * nr + idx) = 0.0;
    }
  }

  // Right-hand side of the translational EoM.
  const kdl::Vector grav_W(0, 0, -st::kGravity);
  const auto eom_trans_right_W = mass * (tar_acc_W - grav_W) - ext_force_W;  // [N]
  d_.head<3>() = cur_rot.inverse(eom_trans_right_W).data;

  // Right-hand side of the rotational EoM.
  const auto eom_rot_right_B = I_B * tar_dgyro_B + cur_gyro_B * (I_B * cur_gyro_B) - ext_torque_B;  // [Nm]
  d_.tail<3>() = eom_rot_right_B.data;

  // FIXME: SQP may fail to converge on tilt hexacopters when the tilt-angle limit exceeds 30 deg.
  // TODO: Since trigonometric functions in the objective function and constraints increase the risk of local solutions,
  // try replacing them with `x`, `y`, and equality constraints.
  // TODO: Consider changes in propeller positions and inertia caused by tilt angles.

  // Update weights.
  const auto linear_scale = mass * kAccelScale;                              // [N]
  const auto angular_scale = (I_B.trace() / 3) * kDGyroScale;                // [Nm]
  const auto thrust_scale = mass * st::kGravity / drone_.prop->numRotors();  // [N]
  Q_.diagonal().head<3>().fill(cfg_.linear_weight / math::sqr(linear_scale));
  Q_.diagonal().tail<3>().fill(cfg_.angular_weight / math::sqr(angular_scale));
  R_.diagonal().fill(cfg_.thrust_weight / math::sqr(thrust_scale));

  // Solve the SQP.
  const auto x_opt = sqp_.solve();
  if (!x_opt) {
    return std::unexpected("SQP failed: " + x_opt.error());
  }

  const auto nr = drone_.prop->numRotors();
  const auto thrusts = thrustDeadband(x_opt->tail(nr));
  const auto tilt_angles = x_opt->head(nr);
  return MixerSolution{ thrusts, tilt_angles };
}

bool SqpMixer::setLinearWeight(double p)
{
  if (p <= 0.0) {
    return false;
  }

  cfg_.linear_weight = p;
  return true;
}

bool SqpMixer::setAngularWeight(double p)
{
  if (p <= 0.0) {
    return false;
  }

  cfg_.angular_weight = p;
  return true;
}

bool SqpMixer::setThrustWeight(double p)
{
  if (p <= 0.0) {
    return false;
  }

  cfg_.thrust_weight = p;
  return true;
}

void SqpMixer::resetTensors()
{
  const auto nr = drone_.prop->numRotors();

  R_.resize(nr);

  B_.conservativeResize(Eigen::NoChange, 3 * nr);
  for (size_t i = 0; i < nr; ++i) {
    B_.block<3, 3>(0, 3 * i).setIdentity();
  }

  N_.conservativeResize(3 * nr, nr);
  N_.setZero();

  dN_dtheta_.resize(3 * nr, nr, nr);
  dN_dtheta_.setZero();

  dN_dtheta_2_.resize(3 * nr, nr, nr, nr);
  dN_dtheta_2_.setZero();

  Ci_.conservativeResize(4 * nr, 2 * nr);
  Ci_.setZero();
  Ci_.block(0, 0, nr, nr).diagonal().setConstant(-1);
  Ci_.block(nr, 0, nr, nr).diagonal().setConstant(1);
  Ci_.block(2 * nr, nr, nr, nr).diagonal().setConstant(-1);
  Ci_.block(3 * nr, nr, nr, nr).diagonal().setConstant(1);

  ci0_.conservativeResize(4 * nr);
  ci0_.setZero();

  dCi_dx_.resize(4 * nr, 2 * nr, 2 * nr);
  dCi_dx_.setZero();

  df_dx_.conservativeResize(2 * nr);

  df_dx_2_.conservativeResize(2 * nr, 2 * nr);
}

std::expected<void, std::string> SqpMixer::initializeSQP()
{
  const auto q0 = kdl::JntArray::Zero(tree_.getNrOfJoints());
  const auto R0 = kdl::Rotation::Identity();
  const auto v0 = kdl::Vector::Zero();

  const auto init_thrusts = np_mixer_.solve(q0, R0, v0, v0, v0);
  if (!init_thrusts) {
    return std::unexpected("Failed to solve the Non-planar mixer: " + init_thrusts.error());
  }

  const auto nr = drone_.prop->numRotors();
  Eigen::VectorXd x0(2 * nr);
  x0.head(nr).setZero();
  x0.tail(nr) = *init_thrusts;

  sqp_.initialize(
    x0,
    std::bind(&self::f, this, std::placeholders::_1),
    std::bind(&self::g, this, std::placeholders::_1),
    std::bind(&self::h, this, std::placeholders::_1),
    std::bind(&self::dfdx, this, std::placeholders::_1),
    std::bind(&self::dgdx, this, std::placeholders::_1),
    std::bind(&self::dhdx, this, std::placeholders::_1),
    std::bind(&self::dFdx, this, std::placeholders::_1),
    std::bind(&self::dGdx, this, std::placeholders::_1),
    std::bind(&self::dHdx, this, std::placeholders::_1));

  return {};
}

double SqpMixer::f(const Eigen::VectorXd& x)
{
  const auto [theta, tau] = splitState(x);
  const auto e = calc_e(theta, tau);
  return 0.5 * (e.transpose() * Q_ * e).value() + 0.5 * (tau.transpose() * R_ * tau).value();
}

Eigen::VectorXd SqpMixer::g(const Eigen::VectorXd& x)
{
  return Ci_ * x + ci0_;
}

Eigen::VectorXd SqpMixer::h(const Eigen::VectorXd&)
{
  return Eigen::VectorXd(0);
}

Eigen::RowVectorXd SqpMixer::dfdx(const Eigen::VectorXd& x)
{
  const auto [theta, tau] = splitState(x);

  const auto C = calc_C(theta);
  const Eigen::Matrix6Xd QC = Q_ * C;

  const auto nr = drone_.prop->numRotors();
  df_dx_.head(nr) = calc_e(theta, tau).transpose() * Q_ * calc_du_dtheta(theta, tau);
  df_dx_.tail(nr) = tau.transpose() * (C.transpose() * QC + R_.toDenseMatrix()) - d_.transpose() * QC;

  return df_dx_;
}

Eigen::MatrixXd SqpMixer::dgdx(const Eigen::VectorXd&)
{
  return Ci_;
}

Eigen::MatrixXd SqpMixer::dhdx(const Eigen::VectorXd&)
{
  return Eigen::MatrixXd(0, stateSize());
}

Eigen::MatrixXd SqpMixer::dFdx(const Eigen::VectorXd& x)
{
  const auto [theta, tau] = splitState(x);

  const Eigen::VectorXd e = calc_e(theta, tau);
  const Eigen::MatrixXd C = calc_C(theta);
  const Eigen::VectorXd Qe = Q_ * e;
  const Eigen::MatrixXd QC = Q_ * C;
  const Eigen::Matrix6Xd du_dtheta = calc_du_dtheta(theta, tau);
  const Eigen::Tensor3Xd du_dtheta_2 = calc_du_dtheta_2(theta, tau);
  const Eigen::Tensor3Xd dC_dtheta = calc_dC_dtheta(theta);

  const auto nr = drone_.prop->numRotors();

  df_dx_2_.topLeftCorner(nr, nr) = Qe.transpose().eval() * du_dtheta_2 + du_dtheta.transpose() * Q_ * du_dtheta;
  df_dx_2_.bottomRightCorner(nr, nr) = C.transpose() * QC + R_.toDenseMatrix();

  const Eigen::MatrixXd d2f_dtheta_dtau = eigen::shuffle(dC_dtheta, { 2, 1, 0 }) * Qe + du_dtheta.transpose() * QC;
  df_dx_2_.topRightCorner(nr, nr) = d2f_dtheta_dtau;
  df_dx_2_.bottomLeftCorner(nr, nr) = d2f_dtheta_dtau.transpose();

  return df_dx_2_;
}

Eigen::Tensor3Xd SqpMixer::dGdx(const Eigen::VectorXd&)
{
  return dCi_dx_;
}

Eigen::Tensor3Xd SqpMixer::dHdx(const Eigen::VectorXd&)
{
  const auto state_size = stateSize();
  return Eigen::Tensor3Xd(0, state_size, state_size);
}

size_t SqpMixer::stateSize() const
{
  return drone_.prop->numRotors() * 2;
}

std::pair<Eigen::VectorXd, Eigen::VectorXd> SqpMixer::splitState(const Eigen::VectorXd& x) const
{
  assert(static_cast<size_t>(x.size()) == stateSize());

  const auto angles = x.head(drone_.prop->numRotors());
  const auto thrusts = x.tail(drone_.prop->numRotors());

  return { angles, thrusts };
}

Eigen::Vector6d SqpMixer::calc_e(const Eigen::VectorXd& theta, const Eigen::VectorXd& tau)
{
  return calc_u(theta, tau) - d_;
}

Eigen::Vector6d SqpMixer::calc_u(const Eigen::VectorXd& theta, const Eigen::VectorXd& tau)
{
  return calc_C(theta) * tau;
}

Eigen::Matrix6Xd SqpMixer::calc_C(const Eigen::VectorXd& theta)
{
  return B_ * calc_N(theta);
}

const Eigen::MatrixXd& SqpMixer::calc_N(const Eigen::VectorXd& theta)
{
  for (const auto& [i, rotor_it] : std::views::enumerate(drone_.prop->rotors)) {
    const auto& rotor = rotor_it.second;
    const auto& elem = tree_.getSegment(rotor->link_name)->second;

    // TODO: Support cases where the tilt joint is not the direct parent of the rotor joint.
    if (!rotor->tilt_joint_name.empty()) {
      const auto& par_elem = elem.parent->second;
      const auto& gpar_elem = par_elem.parent->second;
      const auto& cur_seg = elem.segment;
      const auto& par_seg = par_elem.segment;
      const auto& gpar_seg = gpar_elem.segment;
      const auto& axis_par = cur_seg.joint().axis();
      const auto& R_base2gpar = fk_solver_.getFrame(gpar_seg.name()).M;
      const auto R_gpar2par = par_seg.pose(theta(i)).M;
      const auto axis_B = R_base2gpar * (R_gpar2par * axis_par);
      N_.block<3, 1>(3 * i, i) = axis_B.data;
    }
    else {
      const auto& R_base2par = fk_solver_.getFrame(elem.parent->first).M;
      const auto axis_B = R_base2par * elem.segment.joint().axis();
      N_.block<3, 1>(3 * i, i) = axis_B.data;
    }
  }

  return N_;
}

const Eigen::Tensor3Xd& SqpMixer::calc_dN_dtheta(const Eigen::VectorXd& theta)
{
  for (const auto& [i, rotor_it] : std::views::enumerate(drone_.prop->rotors)) {
    const auto& rotor = rotor_it.second;
    const auto& elem = tree_.getSegment(rotor->link_name)->second;

    if (!rotor->tilt_joint_name.empty()) {
      const auto& par_elem = elem.parent->second;
      const auto& gpar_elem = par_elem.parent->second;
      const auto& cur_seg = elem.segment;
      const auto& par_seg = par_elem.segment;
      const auto& gpar_seg = gpar_elem.segment;
      const auto& axis_par = cur_seg.joint().axis().data;
      const auto& R_base2gpar = fk_solver_.getFrame(gpar_seg.name()).M.data;
      const Eigen::Vector3d dn_dtheta = R_base2gpar * par_seg.rotGrad(theta(i)) * axis_par;
      eigen::setVectorX(dN_dtheta_, dn_dtheta, { 3 * (int)i, (int)i, (int)i });
    }
  }

  return dN_dtheta_;
}

const Eigen::Tensor4Xd& SqpMixer::calc_dN_dtheta_2(const Eigen::VectorXd& theta)
{
  for (const auto& [i, rotor_it] : std::views::enumerate(drone_.prop->rotors)) {
    const auto& rotor = rotor_it.second;
    const auto& elem = tree_.getSegment(rotor->link_name)->second;

    if (!rotor->tilt_joint_name.empty()) {
      const auto& par_elem = elem.parent->second;
      const auto& gpar_elem = par_elem.parent->second;
      const auto& cur_seg = elem.segment;
      const auto& par_seg = par_elem.segment;
      const auto& gpar_seg = gpar_elem.segment;
      const auto& axis_par = cur_seg.joint().axis().data;
      const auto& R_base2gpar = fk_solver_.getFrame(gpar_seg.name()).M.data;
      const Eigen::Vector3d dn_dtheta_2 = R_base2gpar * par_seg.rotGrad2(theta(i)) * axis_par;
      eigen::setVectorX(dN_dtheta_2_, dn_dtheta_2, { 3 * (int)i, (int)i, (int)i, (int)i });
    }
  }

  return dN_dtheta_2_;
}

Eigen::Matrix6Xd SqpMixer::calc_du_dtheta(const Eigen::VectorXd& theta, const Eigen::VectorXd& tau)
{
  return eigen::shuffle(calc_dC_dtheta(theta), { 0, 2, 1 }) * tau;
}

Eigen::Tensor3Xd SqpMixer::calc_du_dtheta_2(const Eigen::VectorXd& theta, const Eigen::VectorXd& tau)
{
  return eigen::shuffle(calc_dC_dtheta_2(theta), { 0, 3, 2, 1 }) * tau;
}

Eigen::Tensor3Xd SqpMixer::calc_dC_dtheta(const Eigen::VectorXd& theta)
{
  return B_ * calc_dN_dtheta(theta);
}

Eigen::Tensor4Xd SqpMixer::calc_dC_dtheta_2(const Eigen::VectorXd& theta)
{
  return B_ * calc_dN_dtheta_2(theta);
}
}  // namespace random_axis_tilt_multicopter
}  // namespace tobas
