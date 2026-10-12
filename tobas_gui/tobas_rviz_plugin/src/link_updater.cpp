// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_rviz_plugin/link_updater.hpp"

namespace tobas
{
namespace rviz
{
LinkUpdater::LinkUpdater(const RobotState& _state) : robot_state_(_state)
{
}

bool LinkUpdater::getLinkTransforms(
  const std::string& _link_name,
  Ogre::Vector3& _visual_position,
  Ogre::Quaternion& _visual_orientation,
  Ogre::Vector3& _collision_position,
  Ogre::Quaternion& _collision_orientation) const
{
  const auto link_model = robot_state_.getRobotModel().getLinkModel(_link_name);
  if (!link_model) {
    return false;
  }

  const auto& transform = robot_state_.getGlobalLinkTransform(link_model);
  const Eigen::Vector3d pos = transform.translation();
  const Eigen::Quaterniond rot(transform.linear());

  _visual_position = Ogre::Vector3(pos.x(), pos.y(), pos.z());
  _visual_orientation = Ogre::Quaternion(rot.w(), rot.x(), rot.y(), rot.z());

  _collision_position = _visual_position;
  _collision_orientation = _visual_orientation;

  return true;
}
}  // namespace rviz
}  // namespace tobas
