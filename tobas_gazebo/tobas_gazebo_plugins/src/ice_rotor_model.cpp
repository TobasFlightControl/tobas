#include <tobas_math/core.hpp>

#include <tobas_gazebo_tools/utils.hpp>

#include "../include/tobas_gazebo_plugins/ice_rotor_model.hpp"
#include "../include/tobas_gazebo_plugins/sdf.hpp"
#include "../include/tobas_gazebo_plugins/utils.hpp"
#include "../include/tobas_gazebo_plugins/common/constants.hpp"

using namespace std;
namespace cmp = gz::sim::components;

namespace gazebo
{
bool ICERotorModel::initialize(
  const sdf::ElementConstPtr& sdf,
  gz::sim::EntityComponentManager& ecm,
  const gz::sim::Model& model)
{
  if (!getSdfParams(sdf))
    return false;

  if (!initializeGazeboObjects(ecm, model))
    return false;

  if (!pitch_filter_.initialize(pitch_time_const_, 0.))
    return false;

  return true;
}

const std::string& ICERotorModel::getLinkName() const
{
  return link_name_;
}

int ICERotorModel::getDirection() const
{
  return direction_;
}

double ICERotorModel::getGearRatio() const
{
  return gear_ratio_;
}

size_t ICERotorModel::getNumBlades() const
{
  return num_blades_;
}

double ICERotorModel::getMotorConst() const
{
  return motor_const_.first * getPitchAngle() + motor_const_.second;
}

double ICERotorModel::getMomentConst() const
{
  return moment_const_;
}

double ICERotorModel::getDragConst() const
{
  return drag_const_;
}

double ICERotorModel::getPitchAngle() const
{
  return pitch_filter_.getValue();
}

double ICERotorModel::getSpeed(const double& engine_speed) const
{
  return engine_speed / gear_ratio_;
}

double ICERotorModel::getVelocity(const double& engine_speed) const
{
  return getSpeed(engine_speed) * direction_;
}

double ICERotorModel::getThrust(const double& engine_speed) const
{
  return getMotorConst() * math::sqr(getSpeed(engine_speed));
}

void ICERotorModel::setTargetPitchAngle(const double& tar_pitch)
{
  if (pitch_range_.inRange(tar_pitch))
  {
    tar_pitch_ = tar_pitch;
  }
  else
  {
    gzwarn << "Target pitch angle of propeller " << link_name_ << " is out of range: " << tar_pitch << " ∉ "
           << pitch_range_ << endl;
    tar_pitch_ = pitch_range_.clamp(tar_pitch);
  }
}

void ICERotorModel::applyWrench(
  gz::sim::EntityComponentManager& ecm,
  const double& engine_speed,
  const gz::math::Vector3d& wind_vel_W)
{
  assert(engine_speed >= 0.);

  // The True Role of Accelerometer Feedback in Quadrotor Control [Martin+, 2010]
  // II-A. Model of a single propeller near hovering

  // Get joint axes
  const auto& local_axis = joint_->Axis(ecm).value().at(0).Xyz();
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
  const auto h_force_W = (-speed * drag_const_) * linvel_perp_W;
  link_->AddWorldWrench(ecm, h_force_W, gz::math::Vector3d::Zero);

  // (2) first term: Rotor drag torque
  const auto torque = moment_const_ * thrust;
  const auto drag_torque_W = (-direction_ * torque) * global_axis;
  parent_link_->AddWorldWrench(ecm, gz::math::Vector3d::Zero, drag_torque_W);
}

void ICERotorModel::updateJointPosition(gz::sim::EntityComponentManager& ecm, const double& engine_pos)
{
  const auto pos = engine_pos / gear_ratio_ * direction_;
  joint_->ResetPosition(ecm, { pos / kRotorSpeedSlowdownSim });
}

bool ICERotorModel::step(const double& dt)
{
  return pitch_filter_.update(tar_pitch_, dt);
}

bool ICERotorModel::getSdfParams(const sdf::ElementConstPtr& sdf)
{
  if (!getSdfParam(sdf, "linkName", link_name_))
    return false;

  if (!getTurningDirection(sdf, direction_))
    return false;

  if (!getSdfParam(sdf, "gearRatio", gear_ratio_))
    return false;
  if (gear_ratio_ <= 0.)
  {
    gzerr << "Gear ratio must be positive." << endl;
    return false;
  }

  if (!getSdfParam(sdf, "numberOfBlades", num_blades_))
    return false;
  if (num_blades_ <= 0)
  {
    gzerr << "The number of blades must be positive." << endl;
    return false;
  }

  if (!getSdfParam(sdf, "motorConstant", motor_const_))
    return false;
  if (motor_const_.first <= 0.)
  {
    gzerr << "The first term of motor constant must be positive." << endl;
    return false;
  }

  if (!getSdfParam(sdf, "momentConstant", moment_const_))
    return false;
  if (moment_const_ <= 0.)
  {
    gzerr << "Moment constant must be positive." << endl;
    return false;
  }

  if (!getSdfParam(sdf, "dragConstant", drag_const_))
    return false;
  if (drag_const_ <= 0.)
  {
    gzerr << "Drag constant must be positive." << endl;
    return false;
  }

  if (!getSdfParam(sdf, "minPitchAngle", pitch_range_.lower))
    return false;
  if (!getSdfParam(sdf, "maxPitchAngle", pitch_range_.upper))
    return false;
  if (!pitch_range_.isValid())
  {
    gzerr << "Pitch range of rotor \"" << link_name_ << "\" is invalid." << endl;
    return false;
  }

  if (!getSdfParam(sdf, "pitchTimeConst", pitch_time_const_))
    return false;
  if (pitch_time_const_ < 0.)
  {
    gzerr << "The time constant of propeller pitch tracking must be non-negative." << endl;
    return false;
  }

  return true;
}

bool ICERotorModel::initializeGazeboObjects(gz::sim::EntityComponentManager& ecm, const gz::sim::Model& model)
{
  // Get joint
  const auto joint_entity = findJointWithChildLink(ecm, link_name_);
  if (!joint_entity.has_value())
  {
    gzerr << "Failed to find the parent joint of rotor link \"" << link_name_ << "\"." << endl;
    return false;
  }
  joint_ = make_shared<gz::sim::Joint>(joint_entity.value());
  if (!joint_->Valid(ecm))
  {
    gzerr << "Failed to find rotor link \"" << link_name_ << "\"." << endl;
    return false;
  }

  // Get joint name
  const auto joint_name = joint_->Name(ecm).value();

  // Check joint type
  const auto joint_type = joint_->Type(ecm).value();
  if (joint_type != sdf::JointType::CONTINUOUS && joint_type != sdf::JointType::REVOLUTE)
  {
    gzerr << "Joint \"" << joint_name << "\" is not a rotating joint." << endl;
    return false;
  }

  // Get child link
  const auto link_entity = model.LinkByName(ecm, link_name_);
  link_ = make_shared<gz::sim::Link>(link_entity);
  if (!link_->Valid(ecm))
  {
    gzerr << "Failed to find the child link \"" << link_name_ << "\"." << endl;
    return false;
  }

  // Get parent link
  const auto parent_link_name = joint_->ChildLinkName(ecm).value();
  const auto parent_link_entity = model.LinkByName(ecm, parent_link_name);
  parent_link_ = make_shared<gz::sim::Link>(parent_link_entity);
  if (!parent_link_->Valid(ecm))
  {
    gzerr << "Failed to find the parent link \"" << parent_link_name << "\"." << endl;
    return false;
  }

  // Create necessary components
  if (!getComponent<cmp::JointAxis>(joint_entity.value(), ecm))
  {
    gzerr << "Failed to get component JointAxis of joint \"" << joint_name << "\"." << endl;
    return false;
  }
  if (!getComponent<cmp::JointVelocity>(joint_entity.value(), ecm))
  {
    gzerr << "Failed to get component JointVelocity of joint \"" << joint_name << "\"." << endl;
    return false;
  }
  if (!getComponent<cmp::WorldPose>(link_entity, ecm))
  {
    gzerr << "Failed to get component WorldPose of link \"" << link_name_ << "\"." << endl;
    return false;
  }
  if (!getComponent<cmp::WorldLinearVelocity>(link_entity, ecm))
  {
    gzerr << "Failed to get component WorldLinearVelocity of link \"" << link_name_ << "\"." << endl;
    return false;
  }

  return true;
}
}  // namespace gazebo
