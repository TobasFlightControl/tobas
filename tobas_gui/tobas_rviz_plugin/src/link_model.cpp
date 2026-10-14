// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_rviz_plugin/link_model.hpp"

#include "tobas_rviz_plugin/joint_model/joint_model.hpp"

namespace tobas::rviz
{
LinkModel::LinkModel(const std::string& _name, size_t _link_index) : name_(_name), link_index_(_link_index)
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

JointModel::ConstSharedPtr LinkModel::getParentJointModel() const
{
  return parent_joint_model_.lock();
}

void LinkModel::setParentJointModel(const JointModel::ConstSharedPtr& _joint)
{
  parent_joint_model_ = _joint;
  is_parent_joint_fixed_ = _joint->getType() == JointModel::kFixed;
}

LinkModel::ConstSharedPtr LinkModel::getParentLinkModel() const
{
  return parent_link_model_.lock();
}

void LinkModel::setParentLinkModel(const ConstSharedPtr& _link)
{
  parent_link_model_ = _link;
}

const std::vector<JointModel::ConstSharedPtr>& LinkModel::getChildJointModels() const
{
  return child_joint_models_;
}

void LinkModel::addChildJointModel(const JointModel::ConstSharedPtr& _joint)
{
  child_joint_models_.push_back(_joint);
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

void LinkModel::setJointOriginTransform(const Eigen::Isometry3d& _transform)
{
  joint_origin_transform_ = _transform;
  joint_origin_transform_is_identity_ =
    joint_origin_transform_.linear().isIdentity() &&
    joint_origin_transform_.translation().norm() < std::numeric_limits<double>::epsilon();
}
}  // namespace tobas::rviz
