// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <memory>
#include <string>
#include <vector>

#include <eigen3/Eigen/Geometry>

namespace tobas
{
namespace rviz
{
class JointModel;

/* A link in the robot kinematic tree. */
class LinkModel
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  using SharedPtr = std::shared_ptr<LinkModel>;
  using ConstSharedPtr = std::shared_ptr<const LinkModel>;

  explicit LinkModel(const std::string& name, size_t link_index);

  const std::string& getName() const;
  size_t getLinkIndex() const;

  std::shared_ptr<const JointModel> getParentJointModel() const;
  void setParentJointModel(const std::shared_ptr<const JointModel>& joint);

  ConstSharedPtr getParentLinkModel() const;
  void setParentLinkModel(const ConstSharedPtr& link);

  const std::vector<std::shared_ptr<const JointModel>>& getChildJointModels() const;
  void addChildJointModel(const std::shared_ptr<const JointModel>& joint);

  const Eigen::Isometry3d& getJointOriginTransform() const;
  bool jointOriginTransformIsIdentity() const;
  bool parentJointIsFixed() const;
  void setJointOriginTransform(const Eigen::Isometry3d& transform);

private:
  const std::string name_;
  const size_t link_index_;

  /* Parent references are non-owning to avoid cycles with the owning child relationships. */
  std::weak_ptr<const JointModel> parent_joint_model_;
  std::weak_ptr<const LinkModel> parent_link_model_;

  std::vector<std::shared_ptr<const JointModel>> child_joint_models_;
  bool is_parent_joint_fixed_ = false;
  bool joint_origin_transform_is_identity_ = true;
  Eigen::Isometry3d joint_origin_transform_ = Eigen::Isometry3d::Identity();
};
}  // namespace rviz
}  // namespace tobas
