// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_y_axis_tilt_vtol_controller/mixer_qp.hpp"

#include <ranges>

#include <tobas_constants/scale.hpp>
#include <tobas_eigen_tools/operators.hpp>
#include <tobas_kdl/conversion/coordinates.hpp>
#include <tobas_std_tools/universal_constants.hpp>
#include <tobas_tools/fixed_wing.hpp>

namespace tobas::y_axis_tilt_vtol
{
namespace
{
constexpr size_t kFixedRotorInequalitySize = 2;
constexpr size_t kTiltRotorInequalitySize = 4;
constexpr size_t kControlSurfaceInequalitySize = 2;
constexpr double kCruiseSpeed = 15.0;  ///< m/s, 固定翼機運動モード計算のための仮のクルーズ時対気速度
constexpr double kMomentThrustRatio =
  1.0 / 3.0;  ///< 実行可能最大モーメントを出すために行って良い最大推力変更比率 3軸あるので1軸制御分には1/3
constexpr double kTiltAngleMargin =
  30.0 / 180.0 * M_PI;  ///< 実行可能最大モーメントを出すために行って良い最大ティルト角変更
///< TODO: 機体ごとに定める
constexpr double kMaxTiltVelocity = 90.0 / 180.0 * M_PI;  ///< rad/s
constexpr double kMaxCsVelocity = M_PI;                   ///< rad/s
}  // namespace

QpMixer::QpMixer(const Drone& drone, const kdl::Tree& tree)
  : super(drone, tree), jnt_parser_(tree), fk_solver_(tree), inertia_solver_(tree)
{
}

void QpMixer::updateInternalDataStructures()
{
  super::updateInternalDataStructures();

  jnt_parser_.updateInternalDataStructures();
  fk_solver_.updateInternalDataStructures();
  inertia_solver_.updateInternalDataStructures();

  // Compute forward kinematics.
  const auto& frames = fk_solver_.jntToCart(kdl::JntArray::Zero(tree_.getNrOfJoints()));

  const auto nr = drone_.prop->numRotors();
  info_.resize(nr);
  state_.resize(nr);
  const auto nc = drone_.fixed_wing->numControlSurfaces();
  cs_info_.resize(nc);
  cs_state_.resize(nc);

  // resize E_, x_, thrust_scale_, scale_
  size_t col = 0;
  size_t inequality_rows = 0;
  for (const auto& [idx, rotor_it] : std::views::enumerate(drone_.prop->rotors)) {
    const auto& rotor = rotor_it.second;
    auto& info = info_[idx];
    info.column = col;
    if (rotor->tilt_joint_name.empty()) {
      info.is_tilt = false;
      info.inequality_row = inequality_rows;
      col += 1;
      inequality_rows += kFixedRotorInequalitySize;
    }
    else {
      info.is_tilt = true;
      info.inequality_row = inequality_rows;
      col += 2;
      inequality_rows += kTiltRotorInequalitySize;
    }
  }
  for (const auto& [idx, cs] : std::views::enumerate(drone_.fixed_wing->control_surfaces)) {
    auto& cs_info = cs_info_[idx];
    cs_info.column = col;
    cs_info.inequality_row = inequality_rows;
    col += 1;
    inequality_rows += kControlSurfaceInequalitySize;
  }

  E_.conservativeResize(Eigen::NoChange, col);
  qp_.resize(col, 0, inequality_rows);
  R_.resize(col);

  // setup info_
  for (const auto& [idx, rotor_it] : std::views::enumerate(drone_.prop->rotors)) {
    const auto& rotor = rotor_it.second;

    // info
    auto& info = info_[idx];
    if (info.is_tilt) {
      const auto& cur_elem = tree_.getSegment(rotor->link_name)->second;
      const auto& par_elem = cur_elem.parent->second;
      const auto& gpar_elem = par_elem.parent->second;

      // Store the sign of the tilt axis.
      const auto& B_T_gpar = frames.at(gpar_elem.segment.name());
      const auto tilt_joint = par_elem.segment.joint();
      const auto tilt_axis = B_T_gpar.M * tilt_joint.axis();  // Tilt axis viewed from the base link.
      const auto tilt_axis_y = tilt_axis.normalized().y();
      assert(math::isClose(std::abs(tilt_axis_y), 1.0));
      info.sign = math::sign(tilt_axis_y);
      info.min_tilt_angle = tilt_joint.lower_limit;
      info.max_tilt_angle = tilt_joint.upper_limit;
    }
  }

  // setup max thrusts
  max_thrust_multicopter_ = 0.0;
  max_thrust_fixed_wing_ = 0.0;
  for (const auto& [link_name, rotor] : drone_.prop->rotors) {
    const auto max_thrust = drone_.prop->maxThrust(link_name);
    max_thrust_multicopter_ += max_thrust;
    if (!rotor->tilt_joint_name.empty()) {
      max_thrust_fixed_wing_ += max_thrust;
    }
  }

  // setup fixed wing related information
  calcModesAngularFreq();

  // setup QP problem
  qp_.setZero();
  for (const auto& [idx, pair] : std::views::enumerate(drone_.prop->rotors)) {
    const auto& rotor = pair.second;
    const auto& info = info_[idx];
    const auto max_thrust = drone_.prop->maxThrust(rotor->link_name);

    if (info.is_tilt) {
      qp_.x_scale(info.column) = max_thrust;
      qp_.x_scale(info.column + 1) = max_thrust;
    }
    else {
      qp_.x_scale(info.column) = max_thrust;
    }
  }
  for (const auto& [idx, pair] : std::views::enumerate(drone_.fixed_wing->control_surfaces)) {
    const auto& cs_info = cs_info_[idx];
    const auto& cs = pair.second;
    const auto& joint = tree_.getSegment(cs.link_name)->second.segment.joint();
    qp_.x_scale(cs_info.column) = joint.upper_limit - joint.lower_limit;
  }
}

void QpMixer::registerInternalDataReset()
{
  require_internal_data_reset_ = true;
}

void QpMixer::setVelocityReliability(const bool& _is_velocity_reliable)
{
  is_velocity_reliable_ = _is_velocity_reliable;
}

std::expected<MixerSolution, std::string> QpMixer::solve(
  const double& dt,
  const double& rho,
  const kdl::JntArray& cur_q,
  const kdl::Vector& cur_gyro_B,
  const kdl::Vector& cur_vel_G,
  const kdl::Vector& tar_dgyro_B,
  const double& thrust,
  const double& thrust_angle)
{
  assert(thrust >= 0);

  // Compute forward kinematics.
  const auto frames = fk_solver_.jntToCart(cur_q);

  // Compute mass properties.
  const auto inertia = inertia_solver_.jntToCart(cur_q);
  const auto& mass = inertia.getMass();
  const auto B_Pos_B2G = inertia.getCOG();
  const auto I_B = inertia.getRotationalInertiaCoG();
  const auto airspeed = is_velocity_reliable_ ? cur_vel_G.norm() : 0.0;

  // 固定翼手動操縦モードからの復帰時
  if (require_internal_data_reset_) {
    updateControlJointPositions(cur_q);
    x_controller_.reset();
    y_controller_.reset();
    z_controller_.reset();
    z2_controller_.reset();
    require_internal_data_reset_ = false;
  }

  // Left-hand side of the EoM matrix equality. tilt rotors + fixed rotors
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
      const auto& B_T_gpar = frames.at(gpar_seg.name());
      const auto B_T_par = B_T_gpar * par_seg.pose(0.0);
      const auto n = B_T_par.M * cur_seg.joint().axis();
      state.alpha = std::atan2(n.x(), n.z());

      // Compute the left-hand side of the equations of motion.
      // When the rotor is dead, force the optimal thrust to zero by setting the transfer from thrust to vehicle motion to zero.
      if (isRotorAlive(rotor->link_name)) {
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
      const auto& B_T_par = frames.at(par_seg.name());
      const auto n = B_T_par.M * cur_seg.joint().axis();
      state.alpha = std::atan2(n.x(), n.z());

      // Update the left-hand side of the equations of motion.
      if (isRotorAlive(rotor->link_name)) {
        const auto& B_Pos_B2P = frames.at(cur_seg.name()).p;
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

  // Left-hand side of the EoM matrix equality. control surfaces
  for (const auto& [idx, cs_it] : std::views::enumerate(drone_.fixed_wing->control_surfaces)) {
    const auto& cs = cs_it.second;
    const auto& cs_info = cs_info_[idx];
    const kdl::Vector effectiveness_frd = kdl::Vector(cs.c_roll_delta, cs.c_pitch_delta, cs.c_yaw_delta);
    kdl::Vector effectiveness_flu;
    kdl::vectorFrdToFlu(effectiveness_frd, effectiveness_flu);
    E_(0, cs_info.column) = dynamicPressure(rho, airspeed) * drone_.fixed_wing->vehicle.wing_surface *
                            drone_.fixed_wing->vehicle.wing_span * effectiveness_flu.x();  // [Nm / rad]
    E_(1, cs_info.column) = dynamicPressure(rho, airspeed) * drone_.fixed_wing->vehicle.wing_surface *
                            drone_.fixed_wing->vehicle.mac * effectiveness_flu.y();  // [Nm / rad]
    E_(2, cs_info.column) = dynamicPressure(rho, airspeed) * drone_.fixed_wing->vehicle.wing_surface *
                            drone_.fixed_wing->vehicle.wing_span * effectiveness_flu.z();  // [Nm / rad]
    E_(3, cs_info.column) = 0.0;                                                           // [N]
    E_(4, cs_info.column) = 0.0;                                                           // [N]
  }

  // Right-hand side of the EoM matrix equality.
  // 出せる最大角速度を求める
  setDesiableRotorInfo(thrust, thrust_angle, E_);
  const auto moment_range = calcMomentRange(E_);
  const Eigen::Vector3d max_angular_acc = I_B.data.diagonal().cwiseInverse().cwiseProduct(moment_range.upper);
  const Eigen::Vector3d min_angular_acc = I_B.data.diagonal().cwiseInverse().cwiseProduct(moment_range.lower);
  // 空力による応答遅れの影響を考慮してPI要素を挟む アンチワインドアップを実施
  x_controller_.setGains(1.0, airspeed / kCruiseSpeed * omega_roll_);
  y_controller_.setGains(1.0, airspeed / kCruiseSpeed * omega_s_);
  z_controller_.setGains(1.0, airspeed / kCruiseSpeed * omega_dutch_roll_);
  z2_controller_.setGains(1.0, airspeed / kCruiseSpeed * omega_dutch_roll_);

  x_controller_.setMaxMin(max_angular_acc.x(), min_angular_acc.x());
  x_controller_.update(tar_dgyro_B.x(), dt);

  y_controller_.setMaxMin(max_angular_acc.y(), min_angular_acc.y());
  y_controller_.update(tar_dgyro_B.y(), dt);

  z_controller_.setMaxMin(max_angular_acc.z(), min_angular_acc.z());
  z_controller_.update(tar_dgyro_B.z(), dt);
  z2_controller_.setMaxMin(max_angular_acc.z(), min_angular_acc.z());
  z2_controller_.update(z_controller_.getValue(), dt);
  // 運動方程式右辺
  const auto eom_rot_right_B = Eigen::Vector3d(
                                 I_B.ixx() * x_controller_.getValue(),
                                 I_B.iyy() * y_controller_.getValue(),
                                 I_B.izz() * z2_controller_.getValue()) +
                               cur_gyro_B * (I_B * cur_gyro_B);  // [Nm]
  f_.head<3>() = eom_rot_right_B.data;
  f_(3) = thrust * std::sin(thrust_angle);
  f_(4) = thrust * std::cos(thrust_angle);

  // Weights for the EoM.
  const auto linear_scale = mass * kAccelScale;                // [N]
  const auto angular_scale = (I_B.trace() / 3) * kDGyroScale;  // [Nm]
  Q_.diagonal().head<3>().fill(cfg_.angular_weight / math::sqr(angular_scale));
  Q_.diagonal().tail<2>().fill(cfg_.linear_weight / math::sqr(linear_scale));

  // Weights for thrust.
  const auto thrust_scale = mass * st::kGravity / drone_.prop->numRotors();  // [N]
  const auto thrust_weight = cfg_.thrust_weight / math::sqr(thrust_scale);
  for (const auto& [idx, rotor_it] : std::views::enumerate(drone_.prop->rotors)) {
    const auto& info = info_[idx];
    size_t size = 0;
    if (info.is_tilt) {
      size = 2;
    }
    else {
      size = 1;
    }
    for (size_t i = 0; i < size; i++) {
      R_.diagonal()(info.column + i) = thrust_weight;
    }
  }

  // Weights for control surface
  for (const auto& [idx, cs_it] : std::views::enumerate(drone_.fixed_wing->control_surfaces)) {
    const auto cs = cs_it.second;
    const auto& cs_info = cs_info_[idx];
    const auto& joint = tree_.getSegment(cs.link_name)->second.segment.joint();
    const auto cs_scale = joint.upper_limit - joint.lower_limit;
    R_.diagonal()(cs_info.column) = cfg_.cs_weight / math::sqr(cs_scale);
  }

  // Objective function.
  qp_.problem.P = E_.transpose() * Q_ * E_ + R_;
  qp_.problem.q = -E_.transpose() * Q_ * f_;

  // Inequality constraints.
  for (const auto& [idx, rotor_it] : std::views::enumerate(drone_.prop->rotors)) {
    const auto& rotor = rotor_it.second;
    const auto& info = info_[idx];
    const auto& state = state_[idx];

    if (info.is_tilt) {
      const auto thrust_angle_lim0 = calcThrustAngle(info.min_tilt_angle, info, state);
      const auto thrust_angle_lim1 = calcThrustAngle(info.max_tilt_angle, info, state);
      const auto physical_min_thrust_angle = std::min(thrust_angle_lim0, thrust_angle_lim1);
      const auto physical_max_thrust_angle = std::max(thrust_angle_lim0, thrust_angle_lim1);
      const auto min_thrust_angle = std::max(physical_min_thrust_angle, state.desiable_thrust_angle - kTiltAngleMargin);
      const auto max_thrust_angle = std::min(physical_max_thrust_angle, state.desiable_thrust_angle + kTiltAngleMargin);
      const auto mid_thrust_angle = 0.5 * (min_thrust_angle + max_thrust_angle);
      const auto view_angle = 0.5 * (max_thrust_angle - min_thrust_angle);
      auto row = info.inequality_row;
      qp_.problem.A(row, info.column) = -std::cos(min_thrust_angle);
      qp_.problem.A(row, info.column + 1) = std::sin(min_thrust_angle);
      qp_.problem.b(row) = 0;
      row++;
      qp_.problem.A(row, info.column) = std::cos(max_thrust_angle);
      qp_.problem.A(row, info.column + 1) = -std::sin(max_thrust_angle);
      qp_.problem.b(row) = 0;
      row++;
      qp_.problem.A(row, info.column) = -std::sin(mid_thrust_angle);
      qp_.problem.A(row, info.column + 1) = -std::cos(mid_thrust_angle);
      qp_.problem.b(row) = -drone_.prop->minThrust(rotor->link_name);
      row++;
      qp_.problem.A(row, info.column) = std::sin(mid_thrust_angle);
      qp_.problem.A(row, info.column + 1) = std::cos(mid_thrust_angle);
      qp_.problem.b(row) = drone_.prop->maxThrust(rotor->link_name) * std::cos(view_angle);
    }
    else {
      qp_.problem.A(info.inequality_row, info.column) = -1;
      qp_.problem.A(info.inequality_row + 1, info.column) = 1;
      qp_.problem.b(info.inequality_row) = 0;
      qp_.problem.b(info.inequality_row + 1) = drone_.prop->maxThrust(rotor->link_name);
    }
  }
  for (const auto& [idx, cs_it] : std::views::enumerate(drone_.fixed_wing->control_surfaces)) {
    const auto& cs_info = cs_info_[idx];
    const auto& cs_state = cs_state_[idx];
    const auto& cs = cs_it.second;
    const auto& joint = tree_.getSegment(cs.link_name)->second.segment.joint();
    qp_.problem.A(cs_info.inequality_row, cs_info.column) = -1;
    qp_.problem.A(cs_info.inequality_row + 1, cs_info.column) = 1;
    qp_.problem.b(cs_info.inequality_row) = -std::max(joint.lower_limit, cs_state.last_angle - kMaxCsVelocity * dt);
    qp_.problem.b(cs_info.inequality_row + 1) = std::min(joint.upper_limit, cs_state.last_angle + kMaxCsVelocity * dt);
  }

  const auto sol = qp_.solve();
  if (!sol) {
    return std::unexpected("QP failed: " + sol.error());
  }
  auto x = sol.value();

  // Convert the solver variables to independently owned rotor commands.
  const auto nr = drone_.prop->numRotors();
  Eigen::VectorXd thrusts(nr);
  Eigen::VectorXd tilt_angles(nr);
  for (const auto& [idx, pair] : std::views::enumerate(drone_.prop->rotors)) {
    const auto& info = info_[idx];
    if (info.is_tilt) {
      thrusts(idx) = x.segment<2>(info.column).norm();
      const auto tx = x(info.column);
      const auto tz = x(info.column + 1);
      auto& state = state_[idx];
      const auto tilt_angle = calcTiltAngle(tx, tz, info, state);
      tilt_angles(idx) = tilt_angle;
    }
    else {
      thrusts(idx) = x(info.column);
      tilt_angles(idx) = 0.0;
    }
  }
  const auto nc = drone_.fixed_wing->numControlSurfaces();
  Eigen::VectorXd cs_deflections(nc);
  for (const auto& [idx, pair] : std::views::enumerate(drone_.fixed_wing->control_surfaces)) {
    const auto cs_info = cs_info_[idx];
    const auto deflection = x(cs_info.column);
    cs_deflections(idx) = deflection;
    auto& cs_state = cs_state_[idx];
    cs_state.last_angle = deflection;
  }

  return MixerSolution{ std::move(thrusts), std::move(tilt_angles), std::move(cs_deflections) };
}

std::expected<double, std::string> QpMixer::getForwardTiltAngle(size_t idx) const
{
  const auto& info = info_.at(idx);
  if (info.is_tilt) {
    const auto& state = state_.at(idx);
    return calcTiltAngle(1.0, 0.0, info, state);
  }
  else {
    return std::unexpected<std::string>("Rotor " + std::to_string(idx) + " is not a tilt rotor.");
  }
}

double QpMixer::calcMaxThrust(const double& thrust_angle) const
{
  const auto sin_chi_0 = std::sin(thrust_angle);
  const auto cos_chi_0 = std::cos(thrust_angle);
  const auto cos_chi_squared =
    std::pow(max_thrust_fixed_wing_ * cos_chi_0, 2) /
    (std::pow(max_thrust_fixed_wing_ * cos_chi_0, 2) + std::pow(max_thrust_multicopter_ * sin_chi_0, 2));
  const auto sin_chi_squared =
    std::pow(max_thrust_multicopter_ * sin_chi_0, 2) /
    (std::pow(max_thrust_fixed_wing_ * cos_chi_0, 2) + std::pow(max_thrust_multicopter_ * sin_chi_0, 2));
  return std::sqrt(
    std::pow(max_thrust_multicopter_, 2) * cos_chi_squared + std::pow(max_thrust_fixed_wing_, 2) * sin_chi_squared);
}

void QpMixer::calcModesAngularFreq()
{
  // Compute mass properties.
  const auto inertia = inertia_solver_.jntToCart(kdl::JntArray::Zero(tree_.getNrOfJoints()));

  // Compute mass properties.
  const auto I_B = inertia.getRotationalInertiaCoG();
  const auto I_xx = I_B.ixx();
  const auto I_yy = I_B.iyy();
  const auto I_zz = I_B.izz();

  // ロールモードの良い近似は\dot{p} - L_p p = 0
  const auto b = drone_.fixed_wing->vehicle.wing_span;
  const auto S = drone_.fixed_wing->vehicle.wing_surface;
  const auto L_p = 0.5 * st::kStandardAirDensity * std::pow(kCruiseSpeed, 2) * S * b *
                   drone_.fixed_wing->aerodynamics.c_roll_p * b / (2.0 * kCruiseSpeed) / I_xx;
  assert(L_p < 0);
  omega_roll_ = -L_p;

  // 短周期モードの良い近似は\ddot{\theta} - (M_q + M_alpha_dot) \dot{\theta} - M_\alpha \theta = 0
  const auto c_mac = drone_.fixed_wing->vehicle.mac;
  const auto M_alpha = 0.5 * st::kStandardAirDensity * std::pow(kCruiseSpeed, 2) * S * c_mac *
                       drone_.fixed_wing->aerodynamics.c_pitch_alpha / I_yy;
  assert(M_alpha < 0);
  omega_s_ = std::sqrt(-M_alpha);

  // ダッチロールモードの粗い近似はs^2 - N_r s + N_beta = 0
  const auto N_beta = 0.5 * st::kStandardAirDensity * std::pow(kCruiseSpeed, 2) * S * b *
                      drone_.fixed_wing->aerodynamics.c_yaw_beta / I_zz;
  assert(N_beta > 0);
  omega_dutch_roll_ = std::sqrt(N_beta);
}

double QpMixer::calcTiltAngle(
  const double& tx,
  const double& tz,
  const StaticRotorLinkInfo& info,
  const DynamicRotorLinkState& state) const
{
  return info.sign * (std::atan2(tx, tz) - state.alpha);
}

double QpMixer::calcThrustAngle(
  const double& tilt_angle,
  const StaticRotorLinkInfo& info,
  const DynamicRotorLinkState& state) const
{
  return tilt_angle / info.sign + state.alpha;
}

void QpMixer::updateControlJointPositions(const kdl::JntArray& cur_q)
{
  // Compute forward kinematics.
  const auto frames = fk_solver_.jntToCart(cur_q);

  // Update tilt angle offsets viewed from the body frame
  for (const auto& [idx, rotor_it] : std::views::enumerate(drone_.prop->rotors)) {
    const auto& rotor = rotor_it.second;
    const auto& info = info_[idx];
    auto& state = state_[idx];

    const auto& cur_elem = tree_.getSegment(rotor->link_name)->second;
    const auto& cur_seg = cur_elem.segment;
    const auto& par_elem = cur_elem.parent->second;
    const auto& par_seg = par_elem.segment;

    if (info.is_tilt) {
      // Update the rotor axis angle relative to the body frame at zero tilt angle.
      const auto& gpar_elem = par_elem.parent->second;
      const auto& gpar_seg = gpar_elem.segment;
      const auto& B_T_gpar = frames.at(gpar_seg.name());
      const auto B_T_par = B_T_gpar * par_seg.pose(0.0);
      const auto n = B_T_par.M * cur_seg.joint().axis();
      state.alpha = std::atan2(n.x(), n.z());
    }
    else {
      // Update the rotor axis angle relative to the body frame.
      const auto& B_T_par = frames.at(par_seg.name());
      const auto n = B_T_par.M * cur_seg.joint().axis();
      state.alpha = std::atan2(n.x(), n.z());
    }
  }

  // Update control surfaces positions
  for (const auto& [idx, cs_it] : std::views::enumerate(drone_.fixed_wing->control_surfaces)) {
    const auto link_name = cs_it.first;
    const auto cs = cs_it.second;
    const auto& joint = tree_.getSegment(cs.link_name)->second.segment.joint();
    const auto kdl_idx = jnt_parser_.jointIndex(joint.name);
    auto& cs_state = cs_state_[idx];
    cs_state.last_angle = cur_q(kdl_idx);
  }
}

void QpMixer::setDesiableRotorInfo(
  const double& _desiable_thrust,
  const double& _thrust_angle,
  const Eigen::Matrix<double, 5, Eigen::Dynamic>& _E)
{
  // calculate desirable thrusts
  const auto thrusts_size = cs_info_.at(0).column;
  Eigen::Vector5d f_unit(0, 0, 0, std::sin(_thrust_angle), std::cos(_thrust_angle));
  const Eigen::VectorXd desiable_unit_thrusts =
    _E.leftCols(thrusts_size).jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(f_unit);

  for (const auto& [idx, rotor_it] : std::views::enumerate(drone_.prop->rotors)) {
    const auto& info = info_[idx];
    auto& state = state_[idx];

    if (info.is_tilt) {
      const auto desiable_thrust_here = _desiable_thrust * desiable_unit_thrusts.segment<2>(info.column).norm();
      const auto desiable_thrust_angle =
        std::atan2(desiable_unit_thrusts(info.column), desiable_unit_thrusts(info.column + 1));
      state.desiable_thrust = desiable_thrust_here;
      state.desiable_thrust_angle = desiable_thrust_angle;
    }
    else {
      state.desiable_thrust = _desiable_thrust * desiable_unit_thrusts(info.column);
    }
  }
}

st::Range<Eigen::Vector3d> QpMixer::calcMomentRange(const Eigen::Matrix<double, 5, Eigen::Dynamic>& E) const
{
  st::Range<Eigen::Vector3d> moment_range(Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());

  for (const auto& [idx, rotor_it] : std::views::enumerate(drone_.prop->rotors)) {
    const auto& rotor = rotor_it.second;
    const auto& info = info_[idx];
    const auto& state = state_[idx];
    const auto desiable_thrust_here = state.desiable_thrust;

    if (info.is_tilt) {
      const auto min_thrust = std::max(0.0, (1.0 - kMomentThrustRatio) * desiable_thrust_here);
      const auto max_thrust =
        std::min((1.0 + kMomentThrustRatio) * desiable_thrust_here, drone_.prop->maxThrust(rotor->link_name));
      const auto desiable_tilt_angle = state.desiable_thrust_angle;
      const auto min_tilt_angle = desiable_tilt_angle - kTiltAngleMargin;
      const auto max_tilt_angle = desiable_tilt_angle + kTiltAngleMargin;

      const Eigen::Vector2d vertex_0(min_thrust * std::sin(max_tilt_angle), min_thrust * std::cos(max_tilt_angle));
      const Eigen::Vector2d vertex_1(max_thrust * std::sin(max_tilt_angle), max_thrust * std::cos(max_tilt_angle));
      const Eigen::Vector2d vertex_2(max_thrust * std::sin(min_tilt_angle), max_thrust * std::cos(min_tilt_angle));
      const Eigen::Vector2d vertex_3(min_thrust * std::sin(min_tilt_angle), min_thrust * std::cos(min_tilt_angle));

      const Eigen::Vector3d candidate_1 = E.block<3, 2>(0, info.column) * vertex_0;
      const Eigen::Vector3d candidate_2 = E.block<3, 2>(0, info.column) * vertex_1;
      const Eigen::Vector3d candidate_3 = E.block<3, 2>(0, info.column) * vertex_2;
      const Eigen::Vector3d candidate_4 = E.block<3, 2>(0, info.column) * vertex_3;

      moment_range.lower += candidate_1.cwiseMin(candidate_2).cwiseMin(candidate_3).cwiseMin(candidate_4);
      moment_range.upper += candidate_1.cwiseMax(candidate_2).cwiseMax(candidate_3).cwiseMax(candidate_4);
    }
    else {
      const auto min_thrust = std::max(0.0, (1.0 - kMomentThrustRatio) * desiable_thrust_here);
      const auto max_thrust =
        std::min((1.0 + kMomentThrustRatio) * desiable_thrust_here, drone_.prop->maxThrust(rotor->link_name));

      const Eigen::Vector3d candidate_1 = E.block<3, 1>(0, info.column) * min_thrust;
      const Eigen::Vector3d candidate_2 = E.block<3, 1>(0, info.column) * max_thrust;

      moment_range.lower += candidate_1.cwiseMin(candidate_2);
      moment_range.upper += candidate_1.cwiseMax(candidate_2);
    }
  }

  for (const auto& [idx, cs_it] : std::views::enumerate(drone_.fixed_wing->control_surfaces)) {
    const auto cs = cs_it.second;
    const auto& cs_info = cs_info_[idx];
    const auto& joint = tree_.getSegment(cs.link_name)->second.segment.joint();

    const Eigen::Vector3d candidate_1 = E.block<3, 1>(0, cs_info.column) * joint.lower_limit;
    const Eigen::Vector3d candidate_2 = E.block<3, 1>(0, cs_info.column) * joint.upper_limit;

    moment_range.lower += candidate_1.cwiseMin(candidate_2);
    moment_range.upper += candidate_1.cwiseMax(candidate_2);
  }

  return moment_range;
}
}  // namespace tobas::y_axis_tilt_vtol
