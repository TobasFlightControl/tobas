// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_rviz_plugin/link_model.hpp"

#include "tobas_rviz_plugin/joint_model/joint_model.hpp"

namespace tobas
{
LinkModel::LinkModel(const std::string& name, size_t link_index) : name_(name), link_index_(link_index)
{
}

const std::string& LinkModel::getName() const
{
  return name_;
}

size_t LinkModel::getLinkIndex() const
{
  return link_index_;
}

const JointModel* LinkModel::getParentJointModel() const
{
  return parent_joint_model_;
}

void LinkModel::setParentJointModel(const JointModel* joint)
{
  parent_joint_model_ = joint;
  is_parent_joint_fixed_ = joint->getType() == JointModel::kFixed;
}

const LinkModel* LinkModel::getParentLinkModel() const
{
  return parent_link_model_;
}

void LinkModel::setParentLinkModel(const LinkModel* link)
{
  parent_link_model_ = link;
}

const std::vector<const JointModel*>& LinkModel::getChildJointModels() const
{
  return child_joint_models_;
}

void LinkModel::addChildJointModel(const JointModel* joint)
{
  child_joint_models_.push_back(joint);
}

const Eigen::Isometry3d& LinkModel::getJointOriginTransform() const
{
  return joint_origin_transform_;
}

bool LinkModel::jointOriginTransformIsIdentity() const
{
  return joint_origin_transform_is_identity_;
}

bool LinkModel::parentJointIsFixed() const
{
  return is_parent_joint_fixed_;
}

void LinkModel::setJointOriginTransform(const Eigen::Isometry3d& transform)
{
  joint_origin_transform_ = transform;
  joint_origin_transform_is_identity_ =
    joint_origin_transform_.linear().isIdentity() &&
    joint_origin_transform_.translation().norm() < std::numeric_limits<double>::epsilon();
}
}  // namespace tobas
