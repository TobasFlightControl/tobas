#pragma once

// OGREは"/usr/include/OGRE"ではなく"/opt/ros/jazzy/opt/rviz_ogre_vendor/include/OGRE"からインクルードする．
// パスに"OGRE/"を含めれば前者，含めなければ後者からインクルードされる．
#include <OgreMatrix4.h>
#include <OgreQuaternion.h>
#include <OgreVector.h>
#include <urdf/model.h>
#include <rviz_default_plugins/robot/link_updater.hpp>

namespace gui
{
namespace urdf_builder
{
namespace ogre
{
class StaticLinkUpdater : public rviz_default_plugins::robot::LinkUpdater
{
public:
  using SharedPtr = std::shared_ptr<StaticLinkUpdater>;

  explicit StaticLinkUpdater(urdf::ModelSharedPtr urdfPtr);

  bool getLinkTransforms(
    const std::string& link_name,
    Ogre::Vector3& visual_position,
    Ogre::Quaternion& visual_orientation,
    Ogre::Vector3& collision_position,
    Ogre::Quaternion& collision_orientation) const override;

  void setLinkStatus(rviz_common::properties::StatusLevel level, const std::string& link_name, const std::string& text)
    const override;

private:
  urdf::ModelSharedPtr urdf_;
  std::unordered_map<std::string, Ogre::Matrix4> transforms_;

  Ogre::Matrix4 findTransform(const urdf::LinkConstSharedPtr& link);
};
}  // namespace ogre
}  // namespace urdf_builder
}  // namespace gui
