// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_rviz_plugin/robot_state.hpp"

#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>

#include "tobas_rviz_plugin/logger.hpp"

namespace tobas
{
namespace rviz
{
namespace
{
rclcpp::Logger getLogger()
{
  return tobas::rviz::getLogger("tobas.robot_state");
}
}  // namespace

RobotState::RobotState(const RobotModel& robot_model) : robot_model_(robot_model)
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
  const std::vector<std::string>& variable_names,
  const std::vector<double>& variable_positions)
{
  assert(variable_names.size() == variable_positions.size());
  for (size_t i = 0; i < variable_names.size(); ++i) {
    const auto index = robot_model_.getVariableIndex(variable_names[i]);
    positions_[index] = variable_positions[i];
    const auto joint = robot_model_.getJointOfVariable(index);
    markDirtyJointTransforms(joint);
    updateMimicJoint(joint);
  }
}

void RobotState::setJointPositions(const std::string& joint_name, const Eigen::Isometry3d& transform)
{
  const auto joint = robot_model_.getJointModel(joint_name);
  if (joint->getVariableCount() == 0) {
    return;
  }
  joint->computeVariablePositions(transform, &positions_.at(joint->getFirstVariableIndex()));
  markDirtyJointTransforms(joint);
  updateMimicJoint(joint);
}

void RobotState::setToDefaultValues()
{
  robot_model_.getVariableDefaultPositions(positions_);
  std::fill(dirty_joint_transforms_.begin(), dirty_joint_transforms_.end(), 1);
  dirty_link_transforms_ = robot_model_.getRootJoint();
}

void RobotState::update(bool force)
{
  if (force) {
    std::fill(dirty_joint_transforms_.begin(), dirty_joint_transforms_.end(), 1);
    dirty_link_transforms_ = robot_model_.getRootJoint();
  }
  updateLinkTransforms();
}

const Eigen::Isometry3d& RobotState::getGlobalLinkTransform(const LinkModel::ConstSharedPtr& link) const
{
  assert(checkLinkTransforms());
  return global_link_transforms_[link->getLinkIndex()];
}

void RobotState::markDirtyJointTransforms(const JointModel::ConstSharedPtr& joint)
{
  dirty_joint_transforms_[joint->getJointIndex()] = 1;
  dirty_link_transforms_ = !dirty_link_transforms_ ? joint : robot_model_.getCommonRoot(dirty_link_transforms_, joint);
}

void RobotState::updateMimicJoint(const JointModel::ConstSharedPtr& joint)
{
  if (joint->getVariableCount() == 0) {
    return;
  }
  const double value = positions_[joint->getFirstVariableIndex()];
  for (const auto& mimic_joint : joint->getMimicRequests()) {
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

void RobotState::updateLinkTransformsInternal(const JointModel::ConstSharedPtr& start)
{
  for (const auto& link : start->getDescendantLinkModels()) {
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

const Eigen::Isometry3d& RobotState::getJointTransform(const JointModel::ConstSharedPtr& joint)
{
  const auto index = joint->getJointIndex();
  if (joint->getVariableCount() == 0) {
    return variable_joint_transforms_[index];
  }

  auto& dirty = dirty_joint_transforms_[index];
  if (dirty) {
    joint->computeTransform(&positions_.at(joint->getFirstVariableIndex()), variable_joint_transforms_[index]);
    dirty = 0;
  }
  return variable_joint_transforms_[index];
}

bool RobotState::checkLinkTransforms() const
{
  if (dirty_link_transforms_) {
    RCLCPP_WARN(getLogger(), "Returning dirty link transforms");
    return false;
  }
  return true;
}
}  // namespace rviz
}  // namespace tobas
