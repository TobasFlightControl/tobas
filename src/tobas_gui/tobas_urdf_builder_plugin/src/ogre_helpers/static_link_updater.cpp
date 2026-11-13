// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_urdf_builder_plugin/ogre_helpers/static_link_updater.hpp"

static inline Ogre::Vector3 URDFVector3ToOgre(const ::urdf::Vector3& _v)
{
  return Ogre::Vector3(_v.x, _v.y, _v.z);
}

static inline Ogre::Quaternion URDFRotationToOgre(const ::urdf::Rotation& _r)
{
  return Ogre::Quaternion(_r.w, _r.x, _r.y, _r.z);
}

namespace tobas::gui::ub::ogre
{
StaticLinkUpdater::StaticLinkUpdater(::urdf::ModelSharedPtr _urdf) : urdf_(std::move(_urdf))
{
  for (const auto& link : urdf_->links_) {
    transforms_[link.first] = findTransform(link.second);
  }
}

bool StaticLinkUpdater::getLinkTransforms(
  const std::string& _link_name,
  Ogre::Vector3& _visual_position,
  Ogre::Quaternion& _visual_orientation,
  Ogre::Vector3& _collision_position,
  Ogre::Quaternion& _collision_orientation) const
{
  const auto link = urdf_->getLink(_link_name);
  if (!link) {
    setLinkStatus(rviz_common::properties::StatusProperty::Error, _link_name, "Transform not found");
    return false;
  }

  const auto& vt = transforms_.at(link->name);
  _visual_position = vt.getTrans();
  _visual_orientation = vt.linear();
  _collision_position = _visual_position;
  _collision_orientation = _visual_orientation;

  setLinkStatus(rviz_common::properties::StatusProperty::Ok, _link_name, "Transform OK");
  return true;
}

void StaticLinkUpdater::setLinkStatus(rviz_common::properties::StatusLevel, const std::string&, const std::string&) const
{
}

Ogre::Matrix4 StaticLinkUpdater::findTransform(const ::urdf::LinkConstSharedPtr& _link)
{
  std::vector<Ogre::Matrix4> matrices;

  auto cur = _link;
  while (cur && urdf_->getRoot() != cur) {
    if (!cur->parent_joint) {
      break;
    }
    const auto& pose = cur->parent_joint->parent_to_joint_origin_transform;
    Ogre::Matrix4 m;
    m.makeTransform(URDFVector3ToOgre(pose.position), Ogre::Vector3(1, 1, 1), URDFRotationToOgre(pose.rotation));

    matrices.push_back(m);
    cur = urdf_->getLink(cur->parent_joint->parent_link_name);
  }

  std::reverse(matrices.begin(), matrices.end());

  matrices.push_back(Ogre::Matrix4::IDENTITY);
  auto result = matrices[0];
  for (size_t i = 1; i < matrices.size(); ++i) {
    result = result * matrices[i];
  }
  return result;
}
}  // namespace tobas::gui::ub::ogre
