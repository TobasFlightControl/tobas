#include "tobas_gazebo_system_plugins/ice_rotor_model.hpp"

#include <tobas_gazebo_tools/utils.hpp>
#include <tobas_math/core.hpp>

#include "tobas_gazebo_system_plugins/common/constants.hpp"
#include "tobas_gazebo_system_plugins/sdf.hpp"

namespace cmp = gz::sim::components;

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
  return getMotorConst(getPitchAngle());
}

double IceRotorModel::getMomentConst() const
{
  return moment_const_;
}

double IceRotorModel::getDragConst() const
{
  return getDragConst(getPitchAngle());
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

void IceRotorModel::setTargetPitchAngle(const double& tar_pitch)
{
  pitch_angle_.setTargetPosition(tar_pitch);
}

void IceRotorModel::applyWrench(
  gz::sim::EntityComponentManager& ecm,
  const double& engine_speed,
  const gz::math::Vector3d& wind_vel_W)
{
  assert(engine_speed >= 0.);

  // The True Role of Accelerometer Feedback in Quadrotor Control [Martin+, 2010]
  // II-A. Model of a single propeller near hovering

  // Get joint axes
  const auto& local_axis = joint_->Axis(ecm).value().front().Xyz();
  const auto global_axis = link_->WorldPose(ecm).value().Rot().RotateVector(local_axis);

  // Compute current state
  const auto speed = getSpeed(engine_speed);
  const auto thrust = getThrust(engine_speed);

  // (1) first term: Thrust Force
  const auto thrust_W = thrust * global_axis;
  link_->AddWorldWrench(ecm, thrust_W, gz::math::Vector3d::Zero);

  // (1) second term: H-force
  const auto linvel_W = link_->WorldLinearVelocity(ecm).value() - wind_vel_W;
  const auto linvel_perp_W = linvel_W - (linvel_W.Dot(global_axis) * global_axis);
  const auto h_force_W = (-speed * getDragConst()) * linvel_perp_W;
  link_->AddWorldWrench(ecm, h_force_W, gz::math::Vector3d::Zero);

  // (2) first term: Rotor drag torque
  const auto torque = moment_const_ * thrust;
  const auto drag_torque_W = (-direction_ * torque) * global_axis;
  parent_link_->AddWorldWrench(ecm, gz::math::Vector3d::Zero, drag_torque_W);
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
  if (gear_ratio_ <= 0.) {
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
  if (pitch_angle_.max_vel < 0.) {
    gzerr << "The maximum propeller pitch angle rate must be non-negative." << std::endl;
    return false;
  }

  if (!getSdfParam(sdf, "motorConstant", motor_const_)) {
    return false;
  }
  if (motor_const_.second <= 0.) {
    gzerr << "The second term of motor constant must be positive." << std::endl;
    return false;
  }
  if (getMotorConst(pitch_angle_.pos_limit.lower) <= 0.) {
    gzerr << "The motor constant at the lower pitch angle limit must be positive." << std::endl;
    return false;
  }

  if (!getSdfParam(sdf, "momentConstant", moment_const_)) {
    return false;
  }
  if (moment_const_ <= 0.) {
    gzerr << "Moment constant must be positive." << std::endl;
    return false;
  }

  if (!getSdfParam(sdf, "dragConstant", drag_const_)) {
    return false;
  }
  if (drag_const_.second < 0.) {
    gzerr << "The second term of drag constant must be non-negative." << std::endl;
    return false;
  }
  if (getDragConst(pitch_angle_.pos_limit.lower) <= 0.) {
    gzerr << "The drag constant at the lower pitch angle limit must be positive." << std::endl;
    return false;
  }

  return true;
}

bool IceRotorModel::initializeGazeboObjects(gz::sim::EntityComponentManager& ecm, const gz::sim::Model& model)
{
  // Get joint
  const auto joint_entity = findJointWithChildLink(ecm, link_name_);
  if (!joint_entity.has_value()) {
    gzerr << "Failed to find the parent joint of rotor link \"" << link_name_ << "\"." << std::endl;
    return false;
  }
  joint_ = std::make_shared<gz::sim::Joint>(joint_entity.value());
  if (!joint_->Valid(ecm)) {
    gzerr << "Failed to find rotor link \"" << link_name_ << "\"." << std::endl;
    return false;
  }

  // Get joint name
  const auto joint_name = joint_->Name(ecm).value();

  // Check joint type
  const auto joint_type = joint_->Type(ecm).value();
  if (joint_type != sdf::JointType::CONTINUOUS && joint_type != sdf::JointType::REVOLUTE) {
    gzerr << "Joint \"" << joint_name << "\" is not a rotating joint." << std::endl;
    return false;
  }

  // Get child link
  const auto link_entity = model.LinkByName(ecm, link_name_);
  link_ = std::make_shared<gz::sim::Link>(link_entity);
  if (!link_->Valid(ecm)) {
    gzerr << "Failed to find the child link \"" << link_name_ << "\"." << std::endl;
    return false;
  }

  // Get parent link
  const auto parent_link_name = joint_->ParentLinkName(ecm).value();
  const auto parent_link_entity = model.LinkByName(ecm, parent_link_name);
  parent_link_ = std::make_shared<gz::sim::Link>(parent_link_entity);
  if (!parent_link_->Valid(ecm)) {
    gzerr << "Failed to find the parent link \"" << parent_link_name << "\"." << std::endl;
    return false;
  }

  // Create necessary components
  if (!getComponent<cmp::JointAxis>(joint_entity.value(), ecm)) {
    gzerr << "Failed to get component JointAxis of joint \"" << joint_name << "\"." << std::endl;
    return false;
  }
  if (!getComponent<cmp::JointVelocity>(joint_entity.value(), ecm)) {
    gzerr << "Failed to get component JointVelocity of joint \"" << joint_name << "\"." << std::endl;
    return false;
  }
  if (!getComponent<cmp::WorldPose>(link_entity, ecm)) {
    gzerr << "Failed to get component WorldPose of link \"" << link_name_ << "\"." << std::endl;
    return false;
  }
  if (!getComponent<cmp::WorldLinearVelocity>(link_entity, ecm)) {
    gzerr << "Failed to get component WorldLinearVelocity of link \"" << link_name_ << "\"." << std::endl;
    return false;
  }

  return true;
}

double IceRotorModel::getMotorConst(double pitch_angle) const
{
  return std::max(motor_const_.first + motor_const_.second * pitch_angle, 0.);
}

double IceRotorModel::getDragConst(double pitch_angle) const
{
  return std::max(drag_const_.first + drag_const_.second * pitch_angle, 0.);
}
}  // namespace gazebo
