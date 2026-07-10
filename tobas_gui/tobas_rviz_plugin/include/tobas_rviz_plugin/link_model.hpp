// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <string>
#include <vector>

#include <eigen3/Eigen/Geometry>

namespace tobas
{
class JointModel;

/* A link in the robot kinematic tree. */
class LinkModel
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  explicit LinkModel(const std::string& name, size_t link_index);

  const std::string& getName() const;
  size_t getLinkIndex() const;

  const JointModel* getParentJointModel() const;
  void setParentJointModel(const JointModel* joint);

  const LinkModel* getParentLinkModel() const;
  void setParentLinkModel(const LinkModel* link);

  const std::vector<const JointModel*>& getChildJointModels() const;
  void addChildJointModel(const JointModel* joint);

  const Eigen::Isometry3d& getJointOriginTransform() const;
  bool jointOriginTransformIsIdentity() const;
  bool parentJointIsFixed() const;
  void setJointOriginTransform(const Eigen::Isometry3d& transform);

private:
  const std::string name_;
  const size_t link_index_;
  const JointModel* parent_joint_model_ = nullptr;
  const LinkModel* parent_link_model_ = nullptr;
  std::vector<const JointModel*> child_joint_models_;
  bool is_parent_joint_fixed_ = false;
  bool joint_origin_transform_is_identity_ = true;
  Eigen::Isometry3d joint_origin_transform_ = Eigen::Isometry3d::Identity();
};
}  // namespace tobas
