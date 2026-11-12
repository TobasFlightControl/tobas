// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_y_axis_tilt_vtol_controller/mixer.hpp"

#include <ranges>

#include <tobas_kdl/conversion/coordinates.hpp>
#include <tobas_std_tools/universal_constants.hpp>
#include <tobas_tools/fixed_wing.hpp>

namespace tobas
{
namespace y_axis_tilt_vtol
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
  info_.resize(nr);
  state_.resize(nr);

  // resize E_, x_, thrust_scale_, scale_
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
  thrusts_size_ = col;
  col += drone_.fixed_wing->numControlSurfaces();

  E_.conservativeResize(Eigen::NoChange, col);
  x_.conservativeResize(col);
  thrust_scale_.conservativeResize(thrusts_size_);
  scale_.conservativeResize(col);

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
      const auto& B_T_gpar = fk_solver_.getFrame(gpar_elem.segment.name());
      const auto tilt_axis = B_T_gpar.M * par_elem.segment.joint().axis();  // Tilt axis viewed from the base link.
      const auto tilt_axis_y = tilt_axis.normalized().y();
      if (!math::isClose(std::abs(tilt_axis_y), 1.0)) {
        std::cerr << "Tilt axis must be parallel to the Y axis." << std::endl;
        return false;
      }
      info.sign = math::sign(tilt_axis_y);
    }
  }

  // setup thrust_scale_ (fixed)
  for (const auto& [idx, rotor_it] : std::views::enumerate(drone_.prop->rotors)) {
    const auto& rotor = rotor_it.second;
    const auto& info = info_[idx];
    size_t size = 0;
    if (info.is_tilt) {
      size = 2;
    }
    else {
      size = 1;
    }
    for (size_t i = 0; i < size; i++) {
      thrust_scale_(info.column + i) = drone_.prop->maxThrust(rotor->link_name);
    }
  }

  // setup scale_ (mutable)
  // skip thrust part, set later
  // setup control surface part
  for (const auto& [idx, pair] : std::views::enumerate(drone_.fixed_wing->control_surfaces)) {
    const auto& cs = pair.second;
    const auto& joint = tree_.getSegment(cs.link_name)->second.segment.joint();
    const auto max_deflection = joint.upper_limit;
    scale_(thrusts_size_ + idx) = max_deflection;
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

  // setup max aerodynamics effectiveness
  max_cs_roll_effectiveness_ = 0.0;
  max_cs_pitch_effectiveness_ = 0.0;
  max_cs_yaw_effectiveness_ = 0.0;
  for (const auto& [_, cs] : drone_.fixed_wing->control_surfaces) {
    const auto& joint = tree_.getSegment(cs.link_name)->second.segment.joint();
    const auto max_deflection = joint.upper_limit;
    if (cs.type == ControlSurfaceType::kAileron) {
      max_cs_roll_effectiveness_ += std::abs(cs.c_roll_delta * max_deflection);
    }
    if (cs.type == ControlSurfaceType::kElevator) {
      max_cs_pitch_effectiveness_ += std::abs(cs.c_pitch_delta * max_deflection);
    }
    if (cs.type == ControlSurfaceType::kRudder) {
      max_cs_yaw_effectiveness_ += std::abs(cs.c_yaw_delta * max_deflection);
    }
  }

  // setup fixed wing related information
  if (!calcModesAngularFreq()) {
    return false;
  }

  return true;
}

double Mixer::calcMaxThrust(const double& thrust_angle) const
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

bool Mixer::solve(
  const double& dt,
  const double& rho,
  const kdl::JntArray& cur_q,
  const kdl::Vector& cur_gyro_B,
  const kdl::Vector& cur_vel_G,
  const kdl::Vector& tar_dgyro_B,
  const double& thrust,
  const double& thrust_angle)
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
  const auto airspeed = cur_vel_G.norm();

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

  // Left-hand side of the EoM matrix equality. control surfaces
  for (const auto& [idx, pair] : std::views::enumerate(drone_.fixed_wing->control_surfaces)) {
    const auto& cs = pair.second;
    const kdl::Vector effectiveness_frd = kdl::Vector(cs.c_roll_delta, cs.c_pitch_delta, cs.c_yaw_delta);
    kdl::Vector effectiveness_flu;
    kdl::vectorFrdToFlu(effectiveness_frd, effectiveness_flu);
    E_(0, thrusts_size_ + idx) = dynamicPressure(rho, airspeed) * drone_.fixed_wing->vehicle.wing_surface *
                                 drone_.fixed_wing->vehicle.wing_span * effectiveness_flu.x();  // [Nm / rad]
    E_(1, thrusts_size_ + idx) = dynamicPressure(rho, airspeed) * drone_.fixed_wing->vehicle.wing_surface *
                                 drone_.fixed_wing->vehicle.mac * effectiveness_flu.y();  // [Nm / rad]
    E_(2, thrusts_size_ + idx) = dynamicPressure(rho, airspeed) * drone_.fixed_wing->vehicle.wing_surface *
                                 drone_.fixed_wing->vehicle.wing_span * effectiveness_flu.z();  // [Nm / rad]
    E_(3, thrusts_size_ + idx) = 0.0;                                                           // [N]
    E_(4, thrusts_size_ + idx) = 0.0;                                                           // [N]
  }

  // Right-hand side of the EoM matrix equality.
  // 理想的にはE x = fを解けば良いが, このようにすると固定されたモータの推力>0の拘束違反や急速なティルト角の変更が行われる
  // まずE_T x_T = [0, 0, 0, sin(thrust_angle), cos(thrust_angle)]を解く thrust_scale_でスケーリング
  f_.head<3>() = Eigen::Vector3d::Zero();
  f_(3) = std::sin(thrust_angle);
  f_(4) = std::cos(thrust_angle);
  const Eigen::MatrixXd S_T = thrust_scale_.asDiagonal();
  const Eigen::VectorXd x_t_base_unit =
    S_T * (E_.leftCols(thrusts_size_) * S_T).jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(f_);
  for (const auto& [idx, info] : std::views::enumerate(info_)) {
    // ティルト可能なモータに対してモーメント入力を考えない際のティルト角を計算しておく, thrust = 0になったときに使用
    if (info.is_tilt) {
      auto& state = state_.at(idx);
      const auto tx = x_t_base_unit(info.column);
      const auto tz = x_t_base_unit(info.column + 1);
      state.default_tilt_angle = calcUnConstrainedTiltAngle(tx, tz, info, state);
    }
  }
  Eigen::VectorXd x_t_base = thrust * x_t_base_unit;
  for (const auto& [idx, info] : std::views::enumerate(info_)) {
    // 固定モータに対して推力>0の拘束を満たすようにする
    if (!info.is_tilt) {
      if (x_t_base(info.column) < 0) {
        x_t_base(info.column) = 0.0;
      }
    }
  }
  // 出せる最大角速度を求める
  Eigen::VectorXd max_thrust_diff = kMomentThrustRatio * x_t_base.cwiseAbs();
  for (const auto& info : info_) {
    if (info.is_tilt) {
      const auto x_index = info.column;
      const auto z_index = x_index + 1;
      max_thrust_diff(x_index) += kMomentTiltAngle * std::abs(x_t_base(z_index));
      max_thrust_diff(z_index) += kMomentTiltAngle * std::abs(x_t_base(x_index));
    }
  }
  const Eigen::Vector3d max_thrust_moment = E_.topLeftCorner(3, thrusts_size_).cwiseAbs() * max_thrust_diff;
  const Eigen::Vector3d max_cs_moment = dynamicPressure(rho, airspeed) * drone_.fixed_wing->vehicle.wing_surface *
                                        Eigen::Vector3d(
                                          drone_.fixed_wing->vehicle.wing_span * max_cs_roll_effectiveness_,
                                          drone_.fixed_wing->vehicle.mac * max_cs_pitch_effectiveness_,
                                          drone_.fixed_wing->vehicle.wing_span * max_cs_yaw_effectiveness_);
  const Eigen::Vector3d max_moment = max_thrust_moment + max_cs_moment;
  const Eigen::Vector3d max_angular_acc = I_B.data.diagonal().cwiseInverse().cwiseProduct(max_moment);
  // 空力による応答遅れの影響を考慮してPI要素を挟む アンチワインドアップを実施
  x_controller_.setGains(1.0, airspeed / kCruiseSpeed * omega_roll_);
  y_controller_.setGains(1.0, airspeed / kCruiseSpeed * omega_s_);
  z_controller_.setGains(1.0, airspeed / kCruiseSpeed * omega_dutch_roll_);
  z2_controller_.setGains(1.0, airspeed / kCruiseSpeed * omega_dutch_roll_);

  x_controller_.setMaxMin(max_angular_acc.x(), -max_angular_acc.x());
  x_controller_.update(tar_dgyro_B.x(), dt);

  y_controller_.setMaxMin(max_angular_acc.y(), -max_angular_acc.y());
  y_controller_.update(tar_dgyro_B.y(), dt);

  z_controller_.setMaxMin(max_angular_acc.z(), -max_angular_acc.z());
  z_controller_.update(tar_dgyro_B.z(), dt);
  z2_controller_.setMaxMin(max_angular_acc.z(), -max_angular_acc.z());
  z2_controller_.update(z_controller_.getValue(), dt);
  // モーメントに関してミキシングする. scale_でスケーリングして固定されたモータの推力>0の拘束違反を防ぐ
  const auto eom_rot_right_B = Eigen::Vector3d(
                                 I_B.ixx() * x_controller_.getValue(),
                                 I_B.iyy() * y_controller_.getValue(),
                                 I_B.izz() * z2_controller_.getValue()) +
                               cur_gyro_B * (I_B * cur_gyro_B);  // [Nm]
  f_.head<3>() = eom_rot_right_B.data - E_.topLeftCorner(3, thrusts_size_) * x_t_base;
  f_(3) = 0;
  f_(4) = 0;
  scale_.head(thrusts_size_) = max_thrust_diff;
  const Eigen::MatrixXd S = scale_.asDiagonal();
  x_ = S * (E_ * S).jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(f_);
  x_.head(thrusts_size_) += x_t_base;
  return true;
}

double Mixer::getDeflection(size_t idx) const
{
  return x_(thrusts_size_ + idx);
}

double Mixer::getThrust(size_t idx) const
{
  const auto& info = info_.at(idx);

  if (info.is_tilt) {
    return thrustDeadband(x_.segment<2>(info.column).norm());
  }
  else {
    return thrustDeadband(x_(info.column));
  }
}

double Mixer::getTiltAngle(size_t idx) const
{
  const auto& info = info_.at(idx);
  const auto& state = state_.at(idx);

  if (info.is_tilt) {
    const auto tx = x_(info.column);
    const auto tz = x_(info.column + 1);
    if (isWithinThrustDeadband(tx) && isWithinThrustDeadband(tz)) {
      return state.default_tilt_angle;
    }
    else {
      return calcUnConstrainedTiltAngle(tx, tz, info, state);
    }
  }
  else {
    std::cerr << "Rotor " << idx << " is not a tilt rotor." << std::endl;
    return 0.0;
  }
}

bool Mixer::calcModesAngularFreq()
{
  // Compute mass properties.
  if (inertia_solver_.jntToCart(kdl::JntArray::Zero(tree_.getNrOfJoints())) < 0) {
    std::cerr << "Inertia solver failed: " << inertia_solver_.errorMessage() << std::endl;
    return false;
  }

  // Compute mass properties.
  const auto& inertia = inertia_solver_.getInertia();
  const auto I_B = inertia.getRotationalInertiaCoG();
  const auto I_xx = I_B.ixx();
  const auto I_yy = I_B.iyy();
  const auto I_zz = I_B.izz();

  // ロールモードの良い近似は\dot{p} - L_p p = 0
  const auto b = drone_.fixed_wing->vehicle.wing_span;
  const auto S = drone_.fixed_wing->vehicle.wing_surface;
  const auto L_p = 0.5 * st::kStandardAirDensity * std::pow(kCruiseSpeed, 2) * S * b *
                   drone_.fixed_wing->aerodynamics.c_roll_p * b / (2.0 * kCruiseSpeed) / I_xx;
  if (L_p > 0) {
    return false;
  }
  omega_roll_ = -L_p;

  // 短周期モードの良い近似は\ddot{\theta} - (M_q + M_alpha_dot) \dot{\theta} - M_\alpha \theta = 0
  const auto c_mac = drone_.fixed_wing->vehicle.mac;
  const auto M_alpha = 0.5 * st::kStandardAirDensity * std::pow(kCruiseSpeed, 2) * S * c_mac *
                       drone_.fixed_wing->aerodynamics.c_pitch_alpha / I_yy;
  const auto omega_squared = -M_alpha;
  if (omega_squared < 0) {
    return false;
  }
  omega_s_ = std::sqrt(omega_squared);

  // ダッチロールモードの粗い近似はs^2 - N_r s + N_beta = 0
  const auto N_beta = 0.5 * st::kStandardAirDensity * std::pow(kCruiseSpeed, 2) * S * b *
                      drone_.fixed_wing->aerodynamics.c_yaw_beta / I_zz;
  if (N_beta < 0) {
    return false;
  }
  omega_dutch_roll_ = std::sqrt(N_beta);
  return true;
}

double Mixer::calcUnConstrainedTiltAngle(
  const double& tx,
  const double& tz,
  const StaticRotorLinkInfo& info,
  const DynamicRotorLinkState& state) const
{
  return info.sign * (std::atan2(tx, tz) - state.alpha);
}

}  // namespace y_axis_tilt_vtol
}  // namespace tobas
