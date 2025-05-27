#include "tobas_rviz_plugin/planning_link_updater.hpp"

#include <Ogre.h>

namespace tobas
{
bool PlanningLinkUpdater::getLinkTransforms(
  const std::string& link_name,
  Ogre::Vector3& visual_position,
  Ogre::Quaternion& visual_orientation,
  Ogre::Vector3& collision_position,
  Ogre::Quaternion& collision_orientation) const
{
  const LinkModel* link_model = robot_state_->getLinkModel(link_name);

  if (!link_model) {
    return false;
  }

  // getGlobalLinkTransform() returns a valid isometry by contract
  const Eigen::Vector3d& robot_visual_position = robot_state_->getGlobalLinkTransform(link_model).translation();
  Eigen::Quaterniond robot_visual_orientation(robot_state_->getGlobalLinkTransform(link_model).linear());
  visual_position = Ogre::Vector3(robot_visual_position.x(), robot_visual_position.y(), robot_visual_position.z());
  visual_orientation = Ogre::Quaternion(
    robot_visual_orientation.w(),
    robot_visual_orientation.x(),
    robot_visual_orientation.y(),
    robot_visual_orientation.z());
  collision_position = visual_position;
  collision_orientation = visual_orientation;

  return true;
}
}  // namespace tobas
