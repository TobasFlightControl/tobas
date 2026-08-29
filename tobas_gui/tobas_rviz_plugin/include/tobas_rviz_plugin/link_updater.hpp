// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <rviz_default_plugins/robot/link_updater.hpp>

#include "./robot_state.hpp"

namespace tobas
{
namespace rviz
{
/* Update the links of an `rviz::Robot` using a `RobotState`. */
class LinkUpdater : public rviz_default_plugins::robot::LinkUpdater
{
public:
  explicit LinkUpdater(const RobotState::ConstSharedPtr& state);

  bool getLinkTransforms(
    const std::string& link_name,
    Ogre::Vector3& visual_position,
    Ogre::Quaternion& visual_orientation,
    Ogre::Vector3& collision_position,
    Ogre::Quaternion& collision_orientation) const override;

private:
  RobotState::ConstSharedPtr robot_state_;
};
}  // namespace rviz
}  // namespace tobas
