#pragma once

#include <unordered_map>
#include <urdf/model.h>
#include <rviz/robot/link_updater.h>
#include <OGRE/OgreMatrix4.h>

namespace urdf_builder
{
namespace ogre_helpers
{
class StaticLinkUpdater : public rviz::LinkUpdater
{
public:
  explicit StaticLinkUpdater(urdf::ModelSharedPtr urdfPtr);

  bool getLinkTransforms(
    const std::string& link_name,
    Ogre::Vector3& visual_position,
    Ogre::Quaternion& visual_orientation,
    Ogre::Vector3& collision_position,
    Ogre::Quaternion& collision_orientation) const override;

  void setLinkStatus(rviz::StatusLevel level, const std::string& link_name, const std::string& text) const override;

private:
  urdf::ModelSharedPtr urdf_;
  std::unordered_map<std::string, Ogre::Matrix4> transforms_;

  Ogre::Matrix4 findTransform(const urdf::Link::ConstSharedPtr& link);
};

using StaticLinkUpdaterPtr = std::shared_ptr<StaticLinkUpdater>;
}  // namespace ogre_helpers
}  // namespace urdf_builder
