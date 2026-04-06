// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_rviz_plugin/robot_state.hpp"

#include <functional>

#include <geometric_shapes/check_isometry.h>
#include <geometric_shapes/shape_operations.h>
#include <rclcpp/rclcpp.hpp>
#include <tf2_eigen/tf2_eigen.hpp>

#include "tobas_rviz_plugin/logger.hpp"

namespace ch = std::chrono;

namespace tobas
{
namespace
{
rclcpp::Logger getLogger()
{
  return tobas::getLogger("tobas.robot_state");
}
}  // namespace

RobotState::RobotState(const RobotModel::ConstSharedPtr& robot_model)
  : robot_model_(robot_model)
  , has_velocity_(false)
  , has_acceleration_(false)
  , has_effort_(false)
  , dirty_link_transforms_(nullptr)
  , dirty_collision_body_transforms_(nullptr)
{
  dirty_link_transforms_ = robot_model_->getRootJoint();
  init();
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

void RobotState::markVelocity()
{
  if (!has_velocity_) {
    has_velocity_ = true;
    std::fill(velocity_.begin(), velocity_.end(), 0.);
  }
}

void RobotState::markAcceleration()
{
  if (!has_acceleration_) {
    has_acceleration_ = true;
    has_effort_ = false;
    std::fill(effort_or_acceleration_.begin(), effort_or_acceleration_.end(), 0.);
  }
}

void RobotState::markEffort()
{
  if (!has_effort_) {
    has_acceleration_ = false;
    has_effort_ = true;
    std::fill(effort_or_acceleration_.begin(), effort_or_acceleration_.end(), 0.);
  }
}

void RobotState::updateMimicJoint(const JointModel* joint)
{
  if (joint->getVariableCount() == 0) {
    return;
  }
  double v = position_[joint->getFirstVariableIndex()];
  for (const JointModel* jm : joint->getMimicRequests()) {
    position_[jm->getFirstVariableIndex()] = jm->getMimicFactor() * v + jm->getMimicOffset();
    markDirtyJointTransforms(jm);
  }
}

void RobotState::setVariablePositions(const double* position)
{
  // assume everything is in order in terms of array lengths (for efficiency reasons)
  memcpy(position_.data(), position, robot_model_->getVariableCount() * sizeof(double));

  // the full state includes mimic joint values, so no need to update mimic here

  // Since all joint values have potentially changed, we will need to recompute all transforms
  std::fill(dirty_joint_transforms_.begin(), dirty_joint_transforms_.end(), 1);
  dirty_link_transforms_ = robot_model_->getRootJoint();
}

void RobotState::setVariablePositions(const std::map<std::string, double>& variable_map)
{
  for (const std::pair<const std::string, double>& it : variable_map) {
    const int index = robot_model_->getVariableIndex(it.first);
    position_[index] = it.second;
    const JointModel* jm = robot_model_->getJointOfVariable(index);
    markDirtyJointTransforms(jm);
    updateMimicJoint(jm);
  }
}

void RobotState::getMissingKeys(
  const std::map<std::string, double>& variable_map,
  std::vector<std::string>& missing_variables) const
{
  missing_variables.clear();
  const std::vector<std::string>& nm = robot_model_->getVariableNames();
  for (const std::string& variable_name : nm) {
    if (variable_map.find(variable_name) == variable_map.end()) {
      if (!robot_model_->getJointOfVariable(variable_name)->getMimic()) {
        missing_variables.push_back(variable_name);
      }
    }
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
    const int index = robot_model_->getVariableIndex(variable_names[i]);
    position_[index] = variable_position[i];
    const JointModel* jm = robot_model_->getJointOfVariable(index);
    markDirtyJointTransforms(jm);
    updateMimicJoint(jm);
  }
}

void RobotState::setVariableVelocities(const std::map<std::string, double>& variable_map)
{
  markVelocity();
  for (const std::pair<const std::string, double>& it : variable_map) {
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

void RobotState::setVariableAccelerations(const std::map<std::string, double>& variable_map)
{
  markAcceleration();
  for (const std::pair<const std::string, double>& it : variable_map) {
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

void RobotState::setVariableEffort(const std::map<std::string, double>& variable_map)
{
  markEffort();
  for (const std::pair<const std::string, double>& it : variable_map) {
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

void RobotState::setJointPositions(const JointModel* joint, const double* position)
{
  if (joint->getVariableCount() == 0) {
    return;
  }
  memcpy(&position_.at(joint->getFirstVariableIndex()), position, joint->getVariableCount() * sizeof(double));
  markDirtyJointTransforms(joint);
  updateMimicJoint(joint);
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
  memcpy(&velocity_.at(joint->getFirstVariableIndex()), velocity, joint->getVariableCount() * sizeof(double));
}

const double* RobotState::getJointPositions(const JointModel* joint) const
{
  if (joint->getVariableCount() == 0) {
    return nullptr;
  }
  return &position_.at(joint->getFirstVariableIndex());
}

const double* RobotState::getJointVelocities(const JointModel* joint) const
{
  if (joint->getVariableCount() == 0) {
    return nullptr;
  }
  return &velocity_.at(joint->getFirstVariableIndex());
}

const double* RobotState::getJointAccelerations(const JointModel* joint) const
{
  if (joint->getVariableCount() == 0) {
    return nullptr;
  }
  return &effort_or_acceleration_.at(joint->getFirstVariableIndex());
}

const double* RobotState::getJointEffort(const JointModel* joint) const
{
  if (joint->getVariableCount() == 0) {
    return nullptr;
  }
  return &effort_or_acceleration_.at(joint->getFirstVariableIndex());
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

  memcpy(
    &effort_or_acceleration_.at(joint->getFirstVariableIndex()), effort, joint->getVariableCount() * sizeof(double));
}

void RobotState::setToDefaultValues()
{
  robot_model_->getVariableDefaultPositions(position_);  // mimic values are updated
  // set velocity & acceleration to 0
  std::fill(velocity_.begin(), velocity_.end(), 0);
  std::fill(effort_or_acceleration_.begin(), effort_or_acceleration_.end(), 0);
  std::fill(dirty_joint_transforms_.begin(), dirty_joint_transforms_.end(), 1);
  dirty_link_transforms_ = robot_model_->getRootJoint();
}

void RobotState::update(bool force)
{
  // make sure we do everything from scratch if needed
  if (force) {
    std::fill(dirty_joint_transforms_.begin(), dirty_joint_transforms_.end(), 1);
    dirty_link_transforms_ = robot_model_->getRootJoint();
  }

  // this actually triggers all needed updates
  updateCollisionBodyTransforms();
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

void RobotState::updateLinkTransformsInternal(const JointModel* start)
{
  for (const LinkModel* link : start->getDescendantLinkModels()) {
    int idx_link = link->getLinkIndex();
    const LinkModel* parent = link->getParentLinkModel();
    if (parent)  // root JointModel will not have a parent
    {
      int idx_parent = parent->getLinkIndex();
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
    else  // is the origin / root / 'model frame'
    {
      const JointModel* root_joint = link->getParentJointModel();
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

const Eigen::Isometry3d& RobotState::getJointTransform(const JointModel* joint)
{
  const int idx = joint->getJointIndex();
  if (joint->getVariableCount() == 0) {
    return variable_joint_transforms_[idx];
  }

  uint8_t& dirty = dirty_joint_transforms_[idx];
  if (dirty) {
    joint->computeTransform(&position_.at(joint->getFirstVariableIndex()), variable_joint_transforms_[idx]);
    dirty = 0;
  }
  return variable_joint_transforms_[idx];
}
}  // namespace tobas
