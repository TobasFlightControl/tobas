// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <rviz_default_plugins/robot/link_updater.hpp>

#include "./robot_state.hpp"

namespace tobas
{
namespace rviz
{
/** Update the links of an `rviz::Robot` using a `RobotState`. */
class LinkUpdater : public rviz_default_plugins::robot::LinkUpdater
{
public:
  explicit LinkUpdater(const RobotState& _state);

  bool getLinkTransforms(
    const std::string& _link_name,
    Ogre::Vector3& _visual_position,
    Ogre::Quaternion& _visual_orientation,
    Ogre::Vector3& _collision_position,
    Ogre::Quaternion& _collision_orientation) const override;

private:
  const RobotState& robot_state_;
};
}  // namespace rviz
}  // namespace tobas
