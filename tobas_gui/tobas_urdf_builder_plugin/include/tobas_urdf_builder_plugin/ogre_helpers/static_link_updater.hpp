// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

// Include OGRE from `/opt/ros/jazzy/opt/rviz_ogre_vendor/include/OGRE`, not `/usr/include/OGRE`.
// If the path includes `OGRE/`, the former is used; otherwise, the latter is used.
#include <OgreMatrix4.h>
#include <OgreQuaternion.h>
#include <OgreVector.h>
#include <urdf/model.h>
#include <rviz_default_plugins/robot/link_updater.hpp>

namespace tobas
{
namespace gui
{
namespace ub
{
namespace ogre
{
class StaticLinkUpdater : public rviz_default_plugins::robot::LinkUpdater
{
public:
  explicit StaticLinkUpdater(::urdf::ModelSharedPtr _urdf_ptr);

  bool getLinkTransforms(
    const std::string& _link_name,
    Ogre::Vector3& _visual_position,
    Ogre::Quaternion& _visual_orientation,
    Ogre::Vector3& _collision_position,
    Ogre::Quaternion& _collision_orientation) const override;

  void setLinkStatus(
    rviz_common::properties::StatusLevel _level,
    const std::string& _link_name,
    const std::string& _text) const override;

private:
  ::urdf::ModelSharedPtr urdf_;
  std::unordered_map<std::string, Ogre::Matrix4> transforms_;

  Ogre::Matrix4 findTransform(const ::urdf::LinkConstSharedPtr& _link);
};
}  // namespace ogre
}  // namespace ub
}  // namespace gui
}  // namespace tobas
