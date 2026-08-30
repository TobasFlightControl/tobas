// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_gazebo_system_plugins/ice_rotor_model.hpp"

#include <tobas_gazebo_tools/utils.hpp>
#include <tobas_math/core.hpp>
#include <tobas_std_tools/check.hpp>
#include <tobas_std_tools/unit_conversions.hpp>

#include "tobas_gazebo_system_plugins/common/constants.hpp"
#include "tobas_gazebo_system_plugins/sdf.hpp"

namespace cmp = gz::sim::components;

namespace tobas
{
namespace gazebo
{
bool IceRotorModel::initialize(
  const sdf::ElementConstPtr& sdf,
  gz::sim::EntityComponentManager& ecm,
  const gz::sim::Model& model)
{
  if (!getSdfParams(sdf)) {
    return false;
  }

  if (!initializeGazeboObjects(ecm, model)) {
    return false;
  }

  return true;
}

const std::string& IceRotorModel::getLinkName() const
{
  return link_name_;
}

int IceRotorModel::getDirection() const
{
  return direction_;
}

double IceRotorModel::getGearRatio() const
{
  return gear_ratio_;
}

size_t IceRotorModel::getNumBlades() const
{
  return num_blades_;
}

double IceRotorModel::getMotorConst() const
{
  return motor_const_.compute(getPitchAngle());
}

double IceRotorModel::getMomentConst() const
{
  return moment_const_.compute(getPitchAngle());
}

double IceRotorModel::getDragConst() const
{
  return drag_const_.compute(getPitchAngle());
}

double IceRotorModel::getPitchAngle() const
{
  return pitch_angle_.getCurrentPosition();
}

double IceRotorModel::getSpeed(const double& engine_speed) const
{
  return engine_speed / gear_ratio_;
}

double IceRotorModel::getVelocity(const double& engine_speed) const
{
  return getSpeed(engine_speed) * direction_;
}

double IceRotorModel::getThrust(const double& engine_speed) const
{
  return getMotorConst() * math::sqr(getSpeed(engine_speed));
}

double IceRotorModel::getTorque(const double& engine_speed) const
{
  return getMomentConst() * getThrust(engine_speed);
}

void IceRotorModel::setTargetPitchAngle(const double& tar_pitch)
{
  pitch_angle_.setTargetPosition(tar_pitch);
}

void IceRotorModel::applyWrench(
  gz::sim::EntityComponentManager& ecm,
  const double& engine_speed,
  const gz::math::Vector3d& wind_vel_W)
{
  assert(engine_speed >= 0.0);

  // Get joint axes.
  const auto& R_W_L = pose_W_->Data().Rot();
  const auto& axis_L = jnt_axis_->Data().Xyz();
  const auto axis_W = R_W_L.RotateVector(axis_L);

  // Coriolis moment (Gyro effect)
  const auto I_W = link_->WorldInertiaMatrix(ecm).value();  // Assume the center of gravity lies on the rotation axis.
  const auto L_W = I_W * (getVelocity(engine_speed) * axis_W);  // Angular momentum of the propeller.
  const auto coriolis_moment_W = -angvel_W_->Data().Cross(L_W);

  // External force: Thrust force
  const auto thrust_force_W = getThrust(engine_speed) * axis_W;

  // External force: H-force
  const auto linvel_rel_W = linvel_W_->Data() - wind_vel_W;
  const auto linvel_perp_W = linvel_rel_W - (linvel_rel_W.Dot(axis_W) * axis_W);
  const auto h_force_W = (-getSpeed(engine_speed) * getDragConst()) * linvel_perp_W;

  // External moment: Drag torque
  const auto drag_moment_W = (-direction_ * getTorque(engine_speed)) * axis_W;

  // Apply wrench.
  link_->AddWorldWrench(ecm, thrust_force_W + h_force_W, gz::math::Vector3d::Zero);
  parent_link_->AddWorldWrench(ecm, gz::math::Vector3d::Zero, coriolis_moment_W + drag_moment_W);  // No inertial force.
}

void IceRotorModel::updateJointPosition(gz::sim::EntityComponentManager& ecm, const double& engine_pos)
{
  const auto pos = engine_pos / gear_ratio_ * direction_;
  joint_->ResetPosition(ecm, { pos / kRotorSpeedSlowdownSim });
}

void IceRotorModel::step(const double& dt)
{
  pitch_angle_.step(dt);
}

bool IceRotorModel::getSdfParams(const sdf::ElementConstPtr& sdf)
{
  if (!getSdfParam(sdf, "linkName", link_name_)) {
    return false;
  }

  if (!getTurningDirection(sdf, direction_)) {
    return false;
  }

  if (!getSdfParam(sdf, "gearRatio", gear_ratio_)) {
    return false;
  }
  if (gear_ratio_ <= 0.0) {
    gzerr << "Gear ratio must be positive." << std::endl;
    return false;
  }

  if (!getSdfParam(sdf, "numberOfBlades", num_blades_)) {
    return false;
  }
  if (num_blades_ <= 0) {
    gzerr << "The number of blades must be positive." << std::endl;
    return false;
  }

  if (!getSdfParam(sdf, "minPitchAngle", pitch_angle_.pos_limit.lower)) {
    return false;
  }
  if (!getSdfParam(sdf, "maxPitchAngle", pitch_angle_.pos_limit.upper)) {
    return false;
  }
  if (!pitch_angle_.pos_limit.isValid()) {
    gzerr << "Propeller pitch angle limit is invalid." << std::endl;
    return false;
  }

  if (!getSdfParam(sdf, "maxPitchAngleRate", pitch_angle_.max_vel)) {
    return false;
  }
  if (pitch_angle_.max_vel < 0.0) {
    gzerr << "The maximum propeller pitch angle rate must be non-negative." << std::endl;
    return false;
  }

  gz::math::Vector2d motor_const;
  if (!getSdfParam(sdf, "motorConstant", motor_const)) {
    return false;
  }
  motor_const_.c0 = motor_const.X();
  motor_const_.c1 = motor_const.Y();
  if (!motor_const_.isValid()) {
    return false;
  }

  gz::math::Vector4d moment_const;
  if (!getSdfParam(sdf, "momentConstant", moment_const)) {
    return false;
  }
  moment_const_.a = moment_const.X();
  moment_const_.b = moment_const.Y();
  moment_const_.c = moment_const.Z();
  moment_const_.phi0 = moment_const.W();
  if (!moment_const_.isValid()) {
    return false;
  }

  gz::math::Vector2d drag_const;
  if (!getSdfParam(sdf, "dragConstant", drag_const)) {
    return false;
  }
  drag_const_.c0 = drag_const.X();
  drag_const_.c1 = drag_const.Y();
  if (!drag_const_.isValid()) {
    return false;
  }

  return true;
}

bool IceRotorModel::initializeGazeboObjects(gz::sim::EntityComponentManager& ecm, const gz::sim::Model& model)
{
  // Get joint.
  const auto joint_entity = findJointWithChildLink(ecm, link_name_);
  if (!joint_entity.has_value()) {
    gzerr << "Failed to find the parent joint of rotor link \"" << link_name_ << "\"." << std::endl;
    return false;
  }
  joint_.emplace(joint_entity.value());
  if (!joint_->Valid(ecm)) {
    gzerr << "Failed to find rotor link \"" << link_name_ << "\"." << std::endl;
    return false;
  }

  // Get joint name.
  const auto joint_name = joint_->Name(ecm).value();

  // Check joint type.
  const auto joint_type = joint_->Type(ecm).value();
  if (joint_type != sdf::JointType::CONTINUOUS && joint_type != sdf::JointType::REVOLUTE) {
    gzerr << "Joint \"" << joint_name << "\" is not a rotating joint." << std::endl;
    return false;
  }

  // Get child link.
  const auto link_entity = model.LinkByName(ecm, link_name_);
  link_.emplace(link_entity);
  if (!link_->Valid(ecm)) {
    gzerr << "Failed to find the child link \"" << link_name_ << "\"." << std::endl;
    return false;
  }

  // Get parent link.
  const auto parent_link_name = joint_->ParentLinkName(ecm).value();
  const auto parent_link_entity = model.LinkByName(ecm, parent_link_name);
  parent_link_.emplace(parent_link_entity);
  if (!parent_link_->Valid(ecm)) {
    gzerr << "Failed to find the parent link \"" << parent_link_name << "\"." << std::endl;
    return false;
  }

  // Create necessary components.
  TOBAS_CHECK(jnt_axis_ = getComponent<cmp::JointAxis>(joint_entity.value(), ecm));
  TOBAS_CHECK(jnt_vel_ = getComponent<cmp::JointVelocity>(joint_entity.value(), ecm));
  TOBAS_CHECK(pose_W_ = getComponent<cmp::WorldPose>(link_entity, ecm));
  TOBAS_CHECK(linvel_W_ = getComponent<cmp::WorldLinearVelocity>(link_entity, ecm));
  TOBAS_CHECK(angvel_W_ = getComponent<cmp::WorldAngularVelocity>(link_entity, ecm));
  TOBAS_CHECK(inertial_ = getComponent<cmp::Inertial>(link_entity, ecm));

  return true;
}

}  // namespace gazebo
}  // namespace tobas
