// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_rviz_plugin/link_updater.hpp"

namespace tobas
{
LinkUpdater::LinkUpdater(const RobotState::ConstSharedPtr& state) : robot_state_(state)
{
}

bool LinkUpdater::getLinkTransforms(
  const std::string& link_name,
  Ogre::Vector3& visual_position,
  Ogre::Quaternion& visual_orientation,
  Ogre::Vector3& collision_position,
  Ogre::Quaternion& collision_orientation) const
{
  const auto link_model = robot_state_->getRobotModel()->getLinkModel(link_name);
  if (!link_model) {
    return false;
  }

  const auto& transform = robot_state_->getGlobalLinkTransform(link_model);
  const Eigen::Vector3d pos = transform.translation();
  const Eigen::Quaterniond rot(transform.linear());

  visual_position = Ogre::Vector3(pos.x(), pos.y(), pos.z());
  visual_orientation = Ogre::Quaternion(rot.w(), rot.x(), rot.y(), rot.z());

  collision_position = visual_position;
  collision_orientation = visual_orientation;

  return true;
}
}  // namespace tobas
