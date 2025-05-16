#pragma once

#include <rviz_default_plugins/robot/robot.hpp>

#include "./class_forward.hpp"
#include "./octomap_render.hpp"
#include "./robot_state.hpp"

namespace tobas
{
TOBAS_CLASS_FORWARD(RenderShapes);             // Defines RenderShapesPtr, ConstPtr, WeakPtr... etc
TOBAS_CLASS_FORWARD(RobotStateVisualization);  // Defines RobotStateVisualizationPtr, ConstPtr, WeakPtr... etc

/* Update the links of an rviz::Robot using a RobotState. */
class RobotStateVisualization
{
public:
  RobotStateVisualization(
    Ogre::SceneNode* root_node,
    rviz_common::DisplayContext* context,
    const std::string& name,
    rviz_common::properties::Property* parent_property);

  rviz_default_plugins::robot::Robot& getRobot()
  {
    return robot_;
  }

  void load(const urdf::ModelInterface& descr, bool visual = true, bool collision = true);
  void clear();

  void update(const RobotStateConstPtr& robot_state);
  void update(const RobotStateConstPtr& robot_state, const std_msgs::msg::ColorRGBA& default_attached_object_color);
  void update(
    const RobotStateConstPtr& robot_state,
    const std_msgs::msg::ColorRGBA& default_attached_object_color,
    const std::map<std::string, std_msgs::msg::ColorRGBA>& color_map);
  void updateKinematicState(const RobotStateConstPtr& robot_state);
  void setDefaultAttachedObjectColor(const std_msgs::msg::ColorRGBA& default_attached_object_color);

  /* update color of all attached object shapes. */
  void updateAttachedObjectColors(const std_msgs::msg::ColorRGBA& attached_object_color);

  bool isVisible() const
  {
    return visible_;
  }

  /**
   * @brief Set the robot as a whole to be visible or not.
   * @param visible Should we be visible?
   */
  void setVisible(bool visible);

  /**
   * @brief Set whether the visual meshes of the robot should be visible.
   * @param visible Whether the visual meshes of the robot should be visible
   */
  void setVisualVisible(bool visible);

  /**
   * @brief Set whether the collision meshes/primitives of the robot should be visible.
   * @param visible Whether the collision meshes/primitives should be visible
   */
  void setCollisionVisible(bool visible);

  void setAlpha(double alpha);

private:
  void updateHelper(
    const RobotStateConstPtr& robot_state,
    const std_msgs::msg::ColorRGBA& default_attached_object_color,
    const std::map<std::string, std_msgs::msg::ColorRGBA>* color_map);
  rviz_default_plugins::robot::Robot robot_;
  RenderShapesPtr render_shapes_;
  std_msgs::msg::ColorRGBA default_attached_object_color_;
  OctreeVoxelRenderMode octree_voxel_render_mode_;
  OctreeVoxelColorMode octree_voxel_color_mode_;

  bool visible_;
  bool visual_visible_;
  bool collision_visible_;
};
}  // namespace tobas
