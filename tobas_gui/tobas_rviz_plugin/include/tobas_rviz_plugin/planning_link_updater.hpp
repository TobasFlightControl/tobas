#pragma once

#include <rviz_default_plugins/robot/link_updater.hpp>

#include "./robot_state.hpp"

namespace tobas
{
/** \brief Update the links of an rviz::Robot using a RobotState */
class PlanningLinkUpdater : public rviz_default_plugins::robot::LinkUpdater
{
public:
  PlanningLinkUpdater(const RobotStateConstPtr& state) : robot_state_(state)
  {
  }

  bool getLinkTransforms(
    const std::string& link_name,
    Ogre::Vector3& visual_position,
    Ogre::Quaternion& visual_orientation,
    Ogre::Vector3& collision_position,
    Ogre::Quaternion& collision_orientation) const override;

private:
  RobotStateConstPtr robot_state_;
};
}  // namespace tobas
