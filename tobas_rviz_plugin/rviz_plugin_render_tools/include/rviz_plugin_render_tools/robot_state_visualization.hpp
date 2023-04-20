#pragma once

#include <rviz/robot/robot.h>
#include <moveit/macros/class_forward.h>
#include <moveit/robot_state/robot_state.h>

namespace moveit_rviz_plugin
{
// MOVEIT_CLASS_FORWARD(RenderShapes);
MOVEIT_CLASS_FORWARD(RobotStateVisualization);

/** \brief Update the links of an rviz::Robot using a robot_state::RobotState */
class RobotStateVisualization
{
public:
  RobotStateVisualization(
    Ogre::SceneNode* root_node,
    rviz::DisplayContext* context,
    const std::string& name,
    rviz::Property* parent_property);

  rviz::Robot& getRobot()
  {
    return robot_;
  }

  void load(const urdf::ModelInterface& descr, bool visual = true, bool collision = true);
  void clear();

  void update(const robot_state::RobotStateConstPtr& kinematic_state);
  void update(
    const robot_state::RobotStateConstPtr& kinematic_state,
    const std_msgs::ColorRGBA& default_attached_object_color);
  void update(
    const robot_state::RobotStateConstPtr& kinematic_state,
    const std_msgs::ColorRGBA& default_attached_object_color,
    const std::map<std::string, std_msgs::ColorRGBA>& color_map);
  void setDefaultAttachedObjectColor(const std_msgs::ColorRGBA& default_attached_object_color);

  /**
   * \brief Set the robot as a whole to be visible or not
   * @param visible Should we be visible?
   */
  void setVisible(bool visible);

  /**
   * \brief Set whether the visual meshes of the robot should be visible
   * @param visible Whether the visual meshes of the robot should be visible
   */
  void setVisualVisible(bool visible);

  /**
   * \brief Set whether the collision meshes/primitives of the robot should be visible
   * @param visible Whether the collision meshes/primitives should be visible
   */
  void setCollisionVisible(bool visible);

  void setAlpha(float alpha);

private:
  void updateHelper(
    const robot_state::RobotStateConstPtr& kinematic_state,
    const std_msgs::ColorRGBA& default_attached_object_color,
    const std::map<std::string, std_msgs::ColorRGBA>* color_map);
  rviz::Robot robot_;
  std_msgs::ColorRGBA default_attached_object_color_;

  bool visible_;
  bool visual_visible_;
  bool collision_visible_;
};
}  // namespace moveit_rviz_plugin
