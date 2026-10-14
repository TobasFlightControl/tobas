// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include "./robot_model.hpp"

namespace tobas::rviz
{
/** Joint positions and the link transforms derived from them. */
class RobotState
{
public:
  explicit RobotState(const RobotModel& _robot_model);

  const RobotModel& getRobotModel() const;

  void
  setVariablePositions(const std::vector<std::string>& _variable_names, const std::vector<double>& _variable_positions);

  void setJointPositions(const std::string& _joint_name, const Eigen::Isometry3d& _transform);

  void setToDefaultValues();
  void update(bool _force = false);

  const Eigen::Isometry3d& getGlobalLinkTransform(const LinkModel::ConstSharedPtr& _link) const;

private:
  void markDirtyJointTransforms(const JointModel::ConstSharedPtr& _joint);
  void updateMimicJoint(const JointModel::ConstSharedPtr& _joint);
  void updateLinkTransforms();
  void updateLinkTransformsInternal(const JointModel::ConstSharedPtr& _start);
  const Eigen::Isometry3d& getJointTransform(const JointModel::ConstSharedPtr& _joint);

  bool checkLinkTransforms() const;

  const RobotModel& robot_model_;
  std::vector<double> positions_;
  JointModel::ConstSharedPtr dirty_link_transforms_;
  std::vector<Eigen::Isometry3d> variable_joint_transforms_;
  std::vector<Eigen::Isometry3d> global_link_transforms_;
  std::vector<uint8_t> dirty_joint_transforms_;
};
}  // namespace tobas::rviz
