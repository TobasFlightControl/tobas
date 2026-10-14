// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_rviz_plugin/robot_state.hpp"

#include <QDebug>

namespace tobas::rviz
{
RobotState::RobotState(const RobotModel& _robot_model) : robot_model_(_robot_model)
{
  dirty_link_transforms_ = robot_model_.getRootJoint();
  positions_.resize(robot_model_.getVariableCount());
  variable_joint_transforms_.resize(robot_model_.getJointModelCount(), Eigen::Isometry3d::Identity());
  global_link_transforms_.resize(robot_model_.getLinkModelCount(), Eigen::Isometry3d::Identity());
  dirty_joint_transforms_.resize(robot_model_.getJointModelCount(), 1);
}

const RobotModel& RobotState::getRobotModel() const
{
  return robot_model_;
}

void RobotState::setVariablePositions(
  const std::vector<std::string>& _variable_names,
  const std::vector<double>& _variable_positions)
{
  assert(_variable_names.size() == _variable_positions.size());
  for (size_t i = 0; i < _variable_names.size(); ++i) {
    const auto index = robot_model_.getVariableIndex(_variable_names[i]);
    positions_[index] = _variable_positions[i];
    const auto joint = robot_model_.getJointOfVariable(index);
    markDirtyJointTransforms(joint);
    updateMimicJoint(joint);
  }
}

void RobotState::setJointPositions(const std::string& _joint_name, const Eigen::Isometry3d& _transform)
{
  const auto joint = robot_model_.getJointModel(_joint_name);
  if (joint->getVariableCount() == 0) {
    return;
  }
  joint->computeVariablePositions(_transform, &positions_.at(joint->getFirstVariableIndex()));
  markDirtyJointTransforms(joint);
  updateMimicJoint(joint);
}

void RobotState::setToDefaultValues()
{
  robot_model_.getVariableDefaultPositions(positions_);
  std::fill(dirty_joint_transforms_.begin(), dirty_joint_transforms_.end(), 1);
  dirty_link_transforms_ = robot_model_.getRootJoint();
}

void RobotState::update(bool _force)
{
  if (_force) {
    std::fill(dirty_joint_transforms_.begin(), dirty_joint_transforms_.end(), 1);
    dirty_link_transforms_ = robot_model_.getRootJoint();
  }
  updateLinkTransforms();
}

const Eigen::Isometry3d& RobotState::getGlobalLinkTransform(const LinkModel::ConstSharedPtr& _link) const
{
  assert(checkLinkTransforms());
  return global_link_transforms_[_link->getLinkIndex()];
}

void RobotState::markDirtyJointTransforms(const JointModel::ConstSharedPtr& _joint)
{
  dirty_joint_transforms_[_joint->getJointIndex()] = 1;
  dirty_link_transforms_ =
    !dirty_link_transforms_ ? _joint : robot_model_.getCommonRoot(dirty_link_transforms_, _joint);
}

void RobotState::updateMimicJoint(const JointModel::ConstSharedPtr& _joint)
{
  if (_joint->getVariableCount() == 0) {
    return;
  }
  const double value = positions_[_joint->getFirstVariableIndex()];
  for (const auto& mimic_joint : _joint->getMimicRequests()) {
    positions_[mimic_joint->getFirstVariableIndex()] =
      mimic_joint->getMimicFactor() * value + mimic_joint->getMimicOffset();
    markDirtyJointTransforms(robot_model_.getJointModel(mimic_joint->getName()));
  }
}

void RobotState::updateLinkTransforms()
{
  if (dirty_link_transforms_) {
    updateLinkTransformsInternal(dirty_link_transforms_);
    dirty_link_transforms_.reset();
  }
}

void RobotState::updateLinkTransformsInternal(const JointModel::ConstSharedPtr& _start)
{
  for (const auto& link : _start->getDescendantLinkModels()) {
    const auto link_index = link->getLinkIndex();
    const auto parent = link->getParentLinkModel();
    if (parent) {
      const auto parent_index = parent->getLinkIndex();
      if (link->parentJointIsFixed()) {
        global_link_transforms_[link_index].affine().noalias() =
          global_link_transforms_[parent_index].affine() * link->getJointOriginTransform().matrix();
      }
      else if (link->jointOriginTransformIsIdentity()) {
        global_link_transforms_[link_index].affine().noalias() =
          global_link_transforms_[parent_index].affine() * getJointTransform(link->getParentJointModel()).matrix();
      }
      else {
        global_link_transforms_[link_index].affine().noalias() =
          global_link_transforms_[parent_index].affine() * link->getJointOriginTransform().matrix() *
          getJointTransform(link->getParentJointModel()).matrix();
      }
    }
    else {
      const auto root_joint = link->getParentJointModel();
      if (root_joint->getVariableCount() == 0) {
        global_link_transforms_[link_index] = Eigen::Isometry3d::Identity();
      }
      else if (link->jointOriginTransformIsIdentity()) {
        global_link_transforms_[link_index] = getJointTransform(root_joint);
      }
      else {
        global_link_transforms_[link_index].affine().noalias() =
          link->getJointOriginTransform().affine() * getJointTransform(root_joint).matrix();
      }
    }
  }
}

const Eigen::Isometry3d& RobotState::getJointTransform(const JointModel::ConstSharedPtr& _joint)
{
  const auto index = _joint->getJointIndex();
  if (_joint->getVariableCount() == 0) {
    return variable_joint_transforms_[index];
  }

  auto& dirty = dirty_joint_transforms_[index];
  if (dirty) {
    _joint->computeTransform(&positions_.at(_joint->getFirstVariableIndex()), variable_joint_transforms_[index]);
    dirty = 0;
  }
  return variable_joint_transforms_[index];
}

bool RobotState::checkLinkTransforms() const
{
  if (dirty_link_transforms_) {
    qWarning() << "Returning dirty link transforms.";
    return false;
  }
  return true;
}
}  // namespace tobas::rviz
