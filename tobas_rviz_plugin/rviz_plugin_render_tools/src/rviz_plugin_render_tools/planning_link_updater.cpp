#include <OgreQuaternion.h>
#include <OgreVector3.h>

#include "../../include/rviz_plugin_render_tools/planning_link_updater.hpp"

using namespace std;
using namespace Eigen;

namespace moveit_rviz_plugin
{
bool PlanningLinkUpdater::getLinkTransforms(
  const string& link_name,
  Ogre::Vector3& visual_position,
  Ogre::Quaternion& visual_orientation,
  Ogre::Vector3& collision_position,
  Ogre::Quaternion& collision_orientation) const
{
  const robot_model::LinkModel* link_model = kinematic_state_->getLinkModel(link_name);

  if (!link_model)
  {
    return false;
  }

  const Vector3d& robot_visual_position =
    kinematic_state_->getGlobalLinkTransform(link_model).translation();
  Quaterniond robot_visual_orientation(
    kinematic_state_->getGlobalLinkTransform(link_model).linear());
  visual_position =
    Ogre::Vector3(robot_visual_position.x(), robot_visual_position.y(), robot_visual_position.z());
  visual_orientation = Ogre::Quaternion(
    robot_visual_orientation.w(), robot_visual_orientation.x(), robot_visual_orientation.y(),
    robot_visual_orientation.z());
  collision_position = visual_position;
  collision_orientation = visual_orientation;

  return true;
}
}  // namespace moveit_rviz_plugin
