#pragma once

#include <rviz/robot/link_updater.h>
#include <moveit/robot_state/robot_state.h>

namespace moveit_rviz_plugin
{
/** \brief Update the links of an rviz::Robot using a robot_state::RobotState */
class PlanningLinkUpdater : public rviz::LinkUpdater
{
public:
  PlanningLinkUpdater(const robot_state::RobotStateConstPtr& state) : kinematic_state_(state)
  {
  }

  virtual bool getLinkTransforms(
    const std::string& link_name,
    Ogre::Vector3& visual_position,
    Ogre::Quaternion& visual_orientation,
    Ogre::Vector3& collision_position,
    Ogre::Quaternion& collision_orientation) const;

private:
  robot_state::RobotStateConstPtr kinematic_state_;
};
}  // namespace moveit_rviz_plugin
