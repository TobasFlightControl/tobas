// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include "./robot_model.hpp"

namespace tobas
{
namespace rviz
{
/* Joint positions and the link transforms derived from them. */
class RobotState
{
public:
  using SharedPtr = std::shared_ptr<RobotState>;
  using ConstSharedPtr = std::shared_ptr<const RobotState>;

  explicit RobotState(const RobotModel::ConstSharedPtr& robot_model);

  const RobotModel::ConstSharedPtr& getRobotModel() const;

  void
  setVariablePositions(const std::vector<std::string>& variable_names, const std::vector<double>& variable_positions);

  void setJointPositions(const std::string& joint_name, const Eigen::Isometry3d& transform);

  void setToDefaultValues();
  void update(bool force = false);

  const Eigen::Isometry3d& getGlobalLinkTransform(const LinkModel* link) const;

private:
  void markDirtyJointTransforms(const JointModel* joint);
  void updateMimicJoint(const JointModel* joint);
  void updateLinkTransforms();
  void updateLinkTransformsInternal(const JointModel* start);
  const Eigen::Isometry3d& getJointTransform(const JointModel* joint);

  bool checkLinkTransforms() const;

  const RobotModel::ConstSharedPtr robot_model_;
  std::vector<double> positions_;
  const JointModel* dirty_link_transforms_ = nullptr;
  std::vector<Eigen::Isometry3d> variable_joint_transforms_;
  std::vector<Eigen::Isometry3d> global_link_transforms_;
  std::vector<uint8_t> dirty_joint_transforms_;
};
}  // namespace rviz
}  // namespace tobas
