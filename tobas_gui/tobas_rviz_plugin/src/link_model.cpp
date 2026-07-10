// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_rviz_plugin/joint_model/joint_model.hpp"

#include "tobas_rviz_plugin/link_model.hpp"

namespace tobas
{
LinkModel::LinkModel(const std::string& name, size_t link_index) : name_(name), link_index_(link_index)
{
  joint_origin_transform_.setIdentity();
}

LinkModel::~LinkModel() = default;

const std::string& LinkModel::getName() const
{
  return name_;
}

size_t LinkModel::getLinkIndex() const
{
  return link_index_;
}

int LinkModel::getFirstCollisionBodyTransformIndex() const
{
  return first_collision_body_transform_index_;
}

void LinkModel::setFirstCollisionBodyTransformIndex(int index)
{
  first_collision_body_transform_index_ = index;
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

const EigenSTL::vector_Isometry3d& LinkModel::getCollisionOriginTransforms() const
{
  return collision_origin_transform_;
}

const std::vector<int>& LinkModel::areCollisionOriginTransformsIdentity() const
{
  return collision_origin_transform_is_identity_;
}

void LinkModel::setCollisionOriginTransforms(const EigenSTL::vector_Isometry3d& origins)
{
  collision_origin_transform_ = origins;
  collision_origin_transform_is_identity_.resize(collision_origin_transform_.size());

  for (size_t i = 0; i < collision_origin_transform_.size(); ++i) {
    collision_origin_transform_is_identity_[i] =
      (collision_origin_transform_[i].linear().isIdentity() &&
       collision_origin_transform_[i].translation().norm() < std::numeric_limits<double>::epsilon()) ?
        1 :
        0;
  }
}

void LinkModel::addAssociatedFixedTransform(const LinkModel* link_model, const Eigen::Isometry3d& transform)
{
  associated_fixed_transforms_[link_model] = transform;
}

void LinkModel::setVisualMesh(
  const std::string& visual_mesh,
  const Eigen::Isometry3d& origin,
  const Eigen::Vector3d& scale)
{
  visual_mesh_filename_ = visual_mesh;
  visual_mesh_origin_ = origin;
  visual_mesh_scale_ = scale;
}
}  // namespace tobas
