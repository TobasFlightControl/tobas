// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_rviz_plugin/robot_state.hpp"

#include <geometric_shapes/check_isometry.h>
#include <geometric_shapes/shape_operations.h>
#include <rclcpp/rclcpp.hpp>
#include <tf2_eigen/tf2_eigen.hpp>

#include "tobas_rviz_plugin/logger.hpp"

namespace tobas
{
namespace
{
rclcpp::Logger getLogger()
{
  return tobas::getLogger("tobas.robot_state");
}
}  // namespace

RobotState::RobotState(const RobotModel::ConstSharedPtr& robot_model) : robot_model_(robot_model)
{
  dirty_link_transforms_ = robot_model_->getRootJoint();
  init();
}

const RobotModel::ConstSharedPtr& RobotState::getRobotModel() const
{
  return robot_model_;
}

double* RobotState::getVariablePositions()
{
  return position_.data();
}

const double* RobotState::getVariablePositions() const
{
  return position_.data();
}

void RobotState::setVariablePositions(const double* position)
{
  // Assume everything is in order in terms of array lengths (for efficiency reasons)
  std::memcpy(position_.data(), position, robot_model_->getVariableCount() * sizeof(double));

  // Since all joint values have potentially changed, we will need to recompute all transforms.
  std::fill(dirty_joint_transforms_.begin(), dirty_joint_transforms_.end(), 1);
  dirty_link_transforms_ = robot_model_->getRootJoint();
}

void RobotState::setVariablePositions(const std::vector<double>& position)
{
  assert(robot_model_->getVariableCount() <= position.size());
  setVariablePositions(&position.front());
}

void RobotState::setVariablePositions(const std::map<std::string, double>& variable_map)
{
  for (const auto& it : variable_map) {
    const auto index = robot_model_->getVariableIndex(it.first);
    position_[index] = it.second;
    const auto jm = robot_model_->getJointOfVariable(index);
    markDirtyJointTransforms(jm);
    updateMimicJoint(jm);
  }
}

void RobotState::setVariablePositions(
  const std::map<std::string, double>& variable_map,
  std::vector<std::string>& missing_variables)
{
  setVariablePositions(variable_map);
  getMissingKeys(variable_map, missing_variables);
}

void RobotState::setVariablePositions(
  const std::vector<std::string>& variable_names,
  const std::vector<double>& variable_position)
{
  for (size_t i = 0; i < variable_names.size(); ++i) {
    const auto index = robot_model_->getVariableIndex(variable_names[i]);
    position_[index] = variable_position[i];
    const auto jm = robot_model_->getJointOfVariable(index);
    markDirtyJointTransforms(jm);
    updateMimicJoint(jm);
  }
}

void RobotState::setVariablePosition(const std::string& variable, double value)
{
  setVariablePosition(robot_model_->getVariableIndex(variable), value);
}

void RobotState::setVariablePosition(int index, double value)
{
  position_[index] = value;
  const auto jm = robot_model_->getJointOfVariable(index);
  if (jm) {
    markDirtyJointTransforms(jm);
    updateMimicJoint(jm);
  }
}

double RobotState::getVariablePosition(const std::string& variable) const
{
  return position_[robot_model_->getVariableIndex(variable)];
}

double RobotState::getVariablePosition(int index) const
{
  return position_[index];
}

bool RobotState::hasVelocities() const
{
  return has_velocity_;
}

double* RobotState::getVariableVelocities()
{
  markVelocity();
  return velocity_.data();
}

const double* RobotState::getVariableVelocities() const
{
  return velocity_.data();
}

void RobotState::setVariableVelocities(const double* velocity)
{
  has_velocity_ = true;

  // Assume everything is in order in terms of array lengths (for efficiency reasons)
  std::memcpy(velocity_.data(), velocity, robot_model_->getVariableCount() * sizeof(double));
}

void RobotState::setVariableVelocities(const std::vector<double>& velocity)
{
  assert(robot_model_->getVariableCount() <= velocity.size());
  setVariableVelocities(&velocity.front());
}

void RobotState::setVariableVelocities(const std::map<std::string, double>& variable_map)
{
  markVelocity();
  for (const auto& it : variable_map) {
    velocity_[robot_model_->getVariableIndex(it.first)] = it.second;
  }
}

void RobotState::setVariableVelocities(
  const std::map<std::string, double>& variable_map,
  std::vector<std::string>& missing_variables)
{
  setVariableVelocities(variable_map);
  getMissingKeys(variable_map, missing_variables);
}

void RobotState::setVariableVelocities(
  const std::vector<std::string>& variable_names,
  const std::vector<double>& variable_velocity)
{
  markVelocity();
  assert(variable_names.size() == variable_velocity.size());
  for (size_t i = 0; i < variable_names.size(); ++i) {
    velocity_[robot_model_->getVariableIndex(variable_names[i])] = variable_velocity[i];
  }
}

void RobotState::setVariableVelocity(const std::string& variable, double value)
{
  setVariableVelocity(robot_model_->getVariableIndex(variable), value);
}

void RobotState::setVariableVelocity(int index, double value)
{
  markVelocity();
  velocity_[index] = value;
}

double RobotState::getVariableVelocity(const std::string& variable) const
{
  return velocity_[robot_model_->getVariableIndex(variable)];
}

double RobotState::getVariableVelocity(int index) const
{
  return velocity_[index];
}

bool RobotState::hasAccelerations() const
{
  return has_acceleration_;
}

double* RobotState::getVariableAccelerations()
{
  markAcceleration();
  return effort_or_acceleration_.data();
}

const double* RobotState::getVariableAccelerations() const
{
  return effort_or_acceleration_.data();
}

void RobotState::setVariableAccelerations(const double* acceleration)
{
  has_acceleration_ = true;
  has_effort_ = false;

  // Assume everything is in order in terms of array lengths (for efficiency reasons)
  std::memcpy(effort_or_acceleration_.data(), acceleration, robot_model_->getVariableCount() * sizeof(double));
}

void RobotState::setVariableAccelerations(const std::vector<double>& acceleration)
{
  assert(robot_model_->getVariableCount() <= acceleration.size());
  setVariableAccelerations(&acceleration.front());
}

void RobotState::setVariableAccelerations(const std::map<std::string, double>& variable_map)
{
  markAcceleration();
  for (const auto& it : variable_map) {
    effort_or_acceleration_[robot_model_->getVariableIndex(it.first)] = it.second;
  }
}

void RobotState::setVariableAccelerations(
  const std::map<std::string, double>& variable_map,
  std::vector<std::string>& missing_variables)
{
  setVariableAccelerations(variable_map);
  getMissingKeys(variable_map, missing_variables);
}

void RobotState::setVariableAccelerations(
  const std::vector<std::string>& variable_names,
  const std::vector<double>& variable_acceleration)
{
  markAcceleration();
  assert(variable_names.size() == variable_acceleration.size());
  for (size_t i = 0; i < variable_names.size(); ++i) {
    effort_or_acceleration_[robot_model_->getVariableIndex(variable_names[i])] = variable_acceleration[i];
  }
}

void RobotState::setVariableAcceleration(const std::string& variable, double value)
{
  setVariableAcceleration(robot_model_->getVariableIndex(variable), value);
}

void RobotState::setVariableAcceleration(int index, double value)
{
  markAcceleration();
  effort_or_acceleration_[index] = value;
}

double RobotState::getVariableAcceleration(const std::string& variable) const
{
  return effort_or_acceleration_[robot_model_->getVariableIndex(variable)];
}

double RobotState::getVariableAcceleration(int index) const
{
  return effort_or_acceleration_[index];
}

bool RobotState::hasEffort() const
{
  return has_effort_;
}

double* RobotState::getVariableEffort()
{
  markEffort();
  return effort_or_acceleration_.data();
}

const double* RobotState::getVariableEffort() const
{
  return effort_or_acceleration_.data();
}

void RobotState::setVariableEffort(const double* effort)
{
  has_effort_ = true;
  has_acceleration_ = false;

  // Assume everything is in order in terms of array lengths (for efficiency reasons)
  std::memcpy(effort_or_acceleration_.data(), effort, robot_model_->getVariableCount() * sizeof(double));
}

void RobotState::setVariableEffort(const std::vector<double>& effort)
{
  assert(robot_model_->getVariableCount() <= effort.size());
  setVariableEffort(&effort.front());
}

void RobotState::setVariableEffort(const std::map<std::string, double>& variable_map)
{
  markEffort();
  for (const auto& it : variable_map) {
    effort_or_acceleration_[robot_model_->getVariableIndex(it.first)] = it.second;
  }
}

void RobotState::setVariableEffort(
  const std::map<std::string, double>& variable_map,
  std::vector<std::string>& missing_variables)
{
  setVariableEffort(variable_map);
  getMissingKeys(variable_map, missing_variables);
}

void RobotState::setVariableEffort(
  const std::vector<std::string>& variable_names,
  const std::vector<double>& variable_effort)
{
  markEffort();
  assert(variable_names.size() == variable_effort.size());
  for (size_t i = 0; i < variable_names.size(); ++i) {
    effort_or_acceleration_[robot_model_->getVariableIndex(variable_names[i])] = variable_effort[i];
  }
}

void RobotState::setVariableEffort(const std::string& variable, double value)
{
  setVariableEffort(robot_model_->getVariableIndex(variable), value);
}

void RobotState::setVariableEffort(int index, double value)
{
  markEffort();
  effort_or_acceleration_[index] = value;
}

double RobotState::getVariableEffort(const std::string& variable) const
{
  return effort_or_acceleration_[robot_model_->getVariableIndex(variable)];
}

double RobotState::getVariableEffort(int index) const
{
  return effort_or_acceleration_[index];
}

void RobotState::setJointPositions(const std::string& joint_name, const double* position)
{
  setJointPositions(robot_model_->getJointModel(joint_name), position);
}

void RobotState::setJointPositions(const std::string& joint_name, const std::vector<double>& position)
{
  setJointPositions(robot_model_->getJointModel(joint_name), &position.front());
}

void RobotState::setJointPositions(const JointModel* joint, const std::vector<double>& position)
{
  setJointPositions(joint, &position.front());
}

void RobotState::setJointPositions(const JointModel* joint, const double* position)
{
  if (joint->getVariableCount() == 0) {
    return;
  }
  std::memcpy(&position_.at(joint->getFirstVariableIndex()), position, joint->getVariableCount() * sizeof(double));
  markDirtyJointTransforms(joint);
  updateMimicJoint(joint);
}

void RobotState::setJointPositions(const std::string& joint_name, const Eigen::Isometry3d& transform)
{
  setJointPositions(robot_model_->getJointModel(joint_name), transform);
}

void RobotState::setJointPositions(const JointModel* joint, const Eigen::Isometry3d& transform)
{
  if (joint->getVariableCount() == 0) {
    return;
  }
  joint->computeVariablePositions(transform, &position_.at(joint->getFirstVariableIndex()));
  markDirtyJointTransforms(joint);
  updateMimicJoint(joint);
}

void RobotState::setJointVelocities(const JointModel* joint, const double* velocity)
{
  if (joint->getVariableCount() == 0) {
    return;
  }
  has_velocity_ = true;
  std::memcpy(&velocity_.at(joint->getFirstVariableIndex()), velocity, joint->getVariableCount() * sizeof(double));
}

void RobotState::setJointEfforts(const JointModel* joint, const double* effort)
{
  if (has_acceleration_) {
    RCLCPP_ERROR(getLogger(), "Unable to set joint efforts because array is being used for accelerations");
    return;
  }
  if (joint->getVariableCount() == 0) {
    return;
  }
  has_effort_ = true;

  std::memcpy(
    &effort_or_acceleration_.at(joint->getFirstVariableIndex()), effort, joint->getVariableCount() * sizeof(double));
}

const double* RobotState::getJointPositions(const std::string& joint_name) const
{
  return getJointPositions(robot_model_->getJointModel(joint_name));
}

const double* RobotState::getJointPositions(const JointModel* joint) const
{
  if (joint->getVariableCount() == 0) {
    return nullptr;
  }
  return &position_.at(joint->getFirstVariableIndex());
}

const double* RobotState::getJointVelocities(const std::string& joint_name) const
{
  return getJointVelocities(robot_model_->getJointModel(joint_name));
}

const double* RobotState::getJointVelocities(const JointModel* joint) const
{
  if (joint->getVariableCount() == 0) {
    return nullptr;
  }
  return &velocity_.at(joint->getFirstVariableIndex());
}

const double* RobotState::getJointAccelerations(const std::string& joint_name) const
{
  return getJointAccelerations(robot_model_->getJointModel(joint_name));
}

const double* RobotState::getJointAccelerations(const JointModel* joint) const
{
  if (joint->getVariableCount() == 0) {
    return nullptr;
  }
  return &effort_or_acceleration_.at(joint->getFirstVariableIndex());
}

const double* RobotState::getJointEffort(const std::string& joint_name) const
{
  return getJointEffort(robot_model_->getJointModel(joint_name));
}

const double* RobotState::getJointEffort(const JointModel* joint) const
{
  if (joint->getVariableCount() == 0) {
    return nullptr;
  }
  return &effort_or_acceleration_.at(joint->getFirstVariableIndex());
}

void RobotState::setVariableValues(const sensor_msgs::msg::JointState& msg)
{
  if (!msg.position.empty()) {
    setVariablePositions(msg.name, msg.position);
  }
  if (!msg.velocity.empty()) {
    setVariableVelocities(msg.name, msg.velocity);
  }
}

void RobotState::setToDefaultValues()
{
  robot_model_->getVariableDefaultPositions(position_);  // mimic values are updated

  // Set velocity & acceleration to 0
  std::fill(velocity_.begin(), velocity_.end(), 0);
  std::fill(effort_or_acceleration_.begin(), effort_or_acceleration_.end(), 0);
  std::fill(dirty_joint_transforms_.begin(), dirty_joint_transforms_.end(), 1);

  dirty_link_transforms_ = robot_model_->getRootJoint();
}

void RobotState::updateCollisionBodyTransforms()
{
  if (dirty_link_transforms_ != nullptr) {
    updateLinkTransforms();
  }

  if (dirty_collision_body_transforms_ != nullptr) {
    const std::vector<const LinkModel*>& links = dirty_collision_body_transforms_->getDescendantLinkModels();
    dirty_collision_body_transforms_ = nullptr;

    for (const LinkModel* link : links) {
      const EigenSTL::vector_Isometry3d& origin_transforms = link->getCollisionOriginTransforms();
      const std::vector<int>& origin_transforms_id = link->areCollisionOriginTransformsIdentity();
      const int index_co = link->getFirstCollisionBodyTransformIndex();
      const int index_l = link->getLinkIndex();
      for (size_t j = 0, end = origin_transforms.size(); j != end; ++j) {
        if (origin_transforms_id[j]) {
          global_collision_body_transforms_[index_co + j] = global_link_transforms_[index_l];
        }
        else {
          global_collision_body_transforms_[index_co + j].affine().noalias() =
            global_link_transforms_[index_l].affine() * origin_transforms[j].matrix();
        }
      }
    }
  }
}

void RobotState::updateLinkTransforms()
{
  if (dirty_link_transforms_ != nullptr) {
    updateLinkTransformsInternal(dirty_link_transforms_);
    if (dirty_collision_body_transforms_) {
      dirty_collision_body_transforms_ =
        robot_model_->getCommonRoot(dirty_collision_body_transforms_, dirty_link_transforms_);
    }
    else {
      dirty_collision_body_transforms_ = dirty_link_transforms_;
    }
    dirty_link_transforms_ = nullptr;
  }
}

void RobotState::update(bool force)
{
  // Make sure we do everything from scratch if needed
  if (force) {
    std::fill(dirty_joint_transforms_.begin(), dirty_joint_transforms_.end(), 1);
    dirty_link_transforms_ = robot_model_->getRootJoint();
  }

  // This actually triggers all needed updates
  updateCollisionBodyTransforms();
}

const Eigen::Isometry3d& RobotState::getGlobalLinkTransform(const std::string& link_name)
{
  return getGlobalLinkTransform(robot_model_->getLinkModel(link_name));
}

const Eigen::Isometry3d& RobotState::getGlobalLinkTransform(const LinkModel* link)
{
  updateLinkTransforms();
  return global_link_transforms_[link->getLinkIndex()];
}

const Eigen::Isometry3d& RobotState::getGlobalLinkTransform(const std::string& link_name) const
{
  return getGlobalLinkTransform(robot_model_->getLinkModel(link_name));
}

const Eigen::Isometry3d& RobotState::getGlobalLinkTransform(const LinkModel* link) const
{
  assert(checkLinkTransforms());
  return global_link_transforms_[link->getLinkIndex()];
}

const Eigen::Isometry3d& RobotState::getJointTransform(const std::string& joint_name)
{
  return getJointTransform(robot_model_->getJointModel(joint_name));
}

const Eigen::Isometry3d& RobotState::getJointTransform(const JointModel* joint)
{
  const auto idx = joint->getJointIndex();
  if (joint->getVariableCount() == 0) {
    return variable_joint_transforms_[idx];
  }

  auto& dirty = dirty_joint_transforms_[idx];
  if (dirty) {
    joint->computeTransform(&position_.at(joint->getFirstVariableIndex()), variable_joint_transforms_[idx]);
    dirty = 0;
  }
  return variable_joint_transforms_[idx];
}

const Eigen::Isometry3d& RobotState::getJointTransform(const std::string& joint_name) const
{
  return getJointTransform(robot_model_->getJointModel(joint_name));
}

const Eigen::Isometry3d& RobotState::getJointTransform(const JointModel* joint) const
{
  assert(checkJointTransforms(joint));
  return variable_joint_transforms_[joint->getJointIndex()];
}

bool RobotState::dirtyJointTransform(const JointModel* joint) const
{
  return dirty_joint_transforms_[joint->getJointIndex()];
}

bool RobotState::dirtyLinkTransforms() const
{
  return dirty_link_transforms_;
}

bool RobotState::dirtyCollisionBodyTransforms() const
{
  return dirty_link_transforms_ || dirty_collision_body_transforms_;
}

bool RobotState::dirty() const
{
  return dirtyCollisionBodyTransforms();
}

void RobotState::init()
{
  variable_joint_transforms_.resize(robot_model_->getJointModelCount(), Eigen::Isometry3d::Identity());
  global_link_transforms_.resize(robot_model_->getLinkModelCount(), Eigen::Isometry3d::Identity());
  global_collision_body_transforms_.resize(robot_model_->getLinkGeometryCount(), Eigen::Isometry3d::Identity());
  dirty_joint_transforms_.resize(robot_model_->getJointModelCount(), 1);
  position_.resize(robot_model_->getVariableCount());
  velocity_.resize(robot_model_->getVariableCount());
  effort_or_acceleration_.resize(robot_model_->getVariableCount());
}

void RobotState::markDirtyJointTransforms(const JointModel* joint)
{
  dirty_joint_transforms_[joint->getJointIndex()] = 1;
  dirty_link_transforms_ = !dirty_link_transforms_ ? joint : robot_model_->getCommonRoot(dirty_link_transforms_, joint);
}

void RobotState::markVelocity()
{
  if (!has_velocity_) {
    has_velocity_ = true;
    std::fill(velocity_.begin(), velocity_.end(), 0.0);
  }
}

void RobotState::markAcceleration()
{
  if (!has_acceleration_) {
    has_acceleration_ = true;
    has_effort_ = false;
    std::fill(effort_or_acceleration_.begin(), effort_or_acceleration_.end(), 0.0);
  }
}

void RobotState::markEffort()
{
  if (!has_effort_) {
    has_acceleration_ = false;
    has_effort_ = true;
    std::fill(effort_or_acceleration_.begin(), effort_or_acceleration_.end(), 0.0);
  }
}

void RobotState::updateMimicJoint(const JointModel* joint)
{
  if (joint->getVariableCount() == 0) {
    return;
  }
  double v = position_[joint->getFirstVariableIndex()];
  for (const auto& jm : joint->getMimicRequests()) {
    position_[jm->getFirstVariableIndex()] = jm->getMimicFactor() * v + jm->getMimicOffset();
    markDirtyJointTransforms(jm);
  }
}

void RobotState::updateLinkTransformsInternal(const JointModel* start)
{
  for (const auto& link : start->getDescendantLinkModels()) {
    const auto idx_link = link->getLinkIndex();
    const auto* parent = link->getParentLinkModel();
    if (parent)  // Root JointModel will not have a parent
    {
      const auto idx_parent = parent->getLinkIndex();
      if (link->parentJointIsFixed()) {  // fixed joint
        global_link_transforms_[idx_link].affine().noalias() =
          global_link_transforms_[idx_parent].affine() * link->getJointOriginTransform().matrix();
      }
      else  // non-fixed joint
      {
        if (link->jointOriginTransformIsIdentity()) {  // Link has identity transform
          global_link_transforms_[idx_link].affine().noalias() =
            global_link_transforms_[idx_parent].affine() * getJointTransform(link->getParentJointModel()).matrix();
        }
        else {  // Link has non-identity transform
          global_link_transforms_[idx_link].affine().noalias() =
            global_link_transforms_[idx_parent].affine() * link->getJointOriginTransform().matrix() *
            getJointTransform(link->getParentJointModel()).matrix();
        }
      }
    }
    else  // is the origin / root / "model frame"
    {
      const auto root_joint = link->getParentJointModel();
      if (root_joint->getVariableCount() == 0) {
        // The root joint doesn't have any variables: avoid calling getJointTransform() on it.
        global_link_transforms_[idx_link] = Eigen::Isometry3d::Identity();
      }
      else if (link->jointOriginTransformIsIdentity()) {
        global_link_transforms_[idx_link] = getJointTransform(root_joint);
      }
      else {
        global_link_transforms_[idx_link].affine().noalias() =
          link->getJointOriginTransform().affine() * getJointTransform(root_joint).matrix();
      }
    }
  }
}

void RobotState::getMissingKeys(
  const std::map<std::string, double>& variable_map,
  std::vector<std::string>& missing_variables) const
{
  missing_variables.clear();
  const auto& nm = robot_model_->getVariableNames();
  for (const auto& variable_name : nm) {
    if (variable_map.find(variable_name) == variable_map.end()) {
      if (!robot_model_->getJointOfVariable(variable_name)->getMimic()) {
        missing_variables.push_back(variable_name);
      }
    }
  }
}

bool RobotState::checkJointTransforms(const JointModel* joint) const
{
  if (dirtyJointTransform(joint)) {
    RCLCPP_WARN(getLogger(), "Returning dirty joint transforms for joint '%s'", joint->getName().c_str());
    return false;
  }
  return true;
}

bool RobotState::checkLinkTransforms() const
{
  if (dirtyLinkTransforms()) {
    RCLCPP_WARN(getLogger(), "Returning dirty link transforms");
    return false;
  }
  return true;
}

bool RobotState::checkCollisionTransforms() const
{
  if (dirtyCollisionBodyTransforms()) {
    RCLCPP_WARN(getLogger(), "Returning dirty collision body transforms");
    return false;
  }
  return true;
}
}  // namespace tobas
