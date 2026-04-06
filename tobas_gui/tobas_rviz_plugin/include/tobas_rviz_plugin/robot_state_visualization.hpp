// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <rviz_default_plugins/robot/robot.hpp>

#include "./octomap_render.hpp"
#include "./render_shapes.hpp"
#include "./robot_state.hpp"

namespace tobas
{
/* Update the links of an rviz::Robot using a RobotState. */
class RobotStateVisualization
{
public:
  explicit RobotStateVisualization(
    Ogre::SceneNode* root_node,
    rviz_common::DisplayContext* context,
    const std::string& name,
    rviz_common::properties::Property* parent_property);

  rviz_default_plugins::robot::Robot& getRobot();

  void load(const urdf::ModelInterface& descr, bool visual = true, bool collision = true);
  void clear();

  void update(const RobotStateConstPtr& robot_state);
  void updateKinematicState(const RobotStateConstPtr& robot_state);
  void setDefaultAttachedObjectColor(const std_msgs::msg::ColorRGBA& color);

  void updateAttachedObjectColors(const std_msgs::msg::ColorRGBA& color);

  bool isVisible() const;
  void setVisible(bool visible);

  void setVisualVisible(bool visible);
  void setCollisionVisible(bool visible);
  void setInertiaVisible(bool visible);

  void setAlpha(double alpha);

private:
  rviz_default_plugins::robot::Robot robot_;
  std::shared_ptr<RenderShapes> render_shapes_;
  Ogre::ColourValue color_;

  OctreeVoxelRenderMode octree_voxel_render_mode_ = kOccupied;
  OctreeVoxelColorMode octree_voxel_color_mode_ = kZAxis;

  bool visible_ = true;
  bool visual_visible_ = true;
  bool collision_visible_ = false;
  bool inertia_visible_ = false;
};
}  // namespace tobas
