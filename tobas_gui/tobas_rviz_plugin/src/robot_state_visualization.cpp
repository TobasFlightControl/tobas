// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_rviz_plugin/robot_state_visualization.hpp"

#include <rviz_common/properties/parse_color.hpp>
#include <rviz_default_plugins/robot/robot_link.hpp>

#include "tobas_rviz_plugin/link_updater.hpp"
#include "tobas_rviz_plugin/logger.hpp"

namespace tobas
{
namespace
{
rclcpp::Logger getLogger()
{
  return tobas::getLogger("tobas.robot_state_visualization");
}
}  // namespace

RobotStateVisualization::RobotStateVisualization(
  Ogre::SceneNode* root_node,
  rviz_common::DisplayContext* context,
  const std::string& name,
  rviz_common::properties::Property* parent_property)
  : robot_(root_node, context, name, parent_property)
{
  color_.r = 0.0f;
  color_.g = 0.7f;
  color_.b = 0.0f;
  color_.a = 1.0f;

  render_shapes_ = std::make_shared<RenderShapes>(context);
}

rviz_default_plugins::robot::Robot& RobotStateVisualization::getRobot()
{
  return robot_;
}

void RobotStateVisualization::load(const urdf::ModelInterface& descr, bool visual, bool collision)
{
  // clear previously loaded model
  clear();

  robot_.load(descr, visual, collision);
  robot_.setVisualVisible(visual_visible_);
  robot_.setCollisionVisible(collision_visible_);
  robot_.setInertiaVisible(inertia_visible_);
  robot_.setVisible(visible_);
}

void RobotStateVisualization::clear()
{
  render_shapes_->clear();
  robot_.clear();
}

void RobotStateVisualization::update(const RobotStateConstPtr& robot_state)
{
  robot_.update(LinkUpdater(robot_state));
  render_shapes_->clear();

  std::vector<const AttachedBody*> attached_bodies;
  robot_state->getAttachedBodies(attached_bodies);
  for (const AttachedBody* attached_body : attached_bodies) {
    const auto alpha = robot_.getAlpha();
    rviz_default_plugins::robot::RobotLink* link = robot_.getLink(attached_body->getAttachedLinkName());
    if (!link) {
      RCLCPP_ERROR_STREAM(getLogger(), "Link " << attached_body->getAttachedLinkName() << " not found in rviz::Robot.");
      continue;
    }
    const auto& ab_t = attached_body->getShapePosesInLinkFrame();
    const auto& ab_shapes = attached_body->getShapes();
    for (size_t j = 0; j < ab_shapes.size(); ++j) {
      render_shapes_->renderShape(
        link->getVisualNode(),
        ab_shapes[j].get(),
        ab_t[j],
        octree_voxel_render_mode_,
        octree_voxel_color_mode_,
        color_,
        alpha);
      render_shapes_->renderShape(
        link->getCollisionNode(),
        ab_shapes[j].get(),
        ab_t[j],
        octree_voxel_render_mode_,
        octree_voxel_color_mode_,
        color_,
        alpha);
    }
  }

  robot_.setVisualVisible(visual_visible_);
  robot_.setCollisionVisible(collision_visible_);
  robot_.setInertiaVisible(inertia_visible_);
  robot_.setVisible(visible_);
}

void RobotStateVisualization::updateKinematicState(const RobotStateConstPtr& robot_state)
{
  robot_.update(LinkUpdater(robot_state));
}

void RobotStateVisualization::updateAttachedObjectColors(const std_msgs::msg::ColorRGBA& color)
{
  render_shapes_->updateShapeColors(color.r, color.g, color.b, robot_.getAlpha());
}

void RobotStateVisualization::setDefaultAttachedObjectColor(const std_msgs::msg::ColorRGBA& color)
{
  color_.r = color.r;
  color_.g = color.g;
  color_.b = color.b;
  color_.a = color.a;
}

bool RobotStateVisualization::isVisible() const
{
  return visible_;
}

void RobotStateVisualization::setVisible(bool visible)
{
  visible_ = visible;
  robot_.setVisible(visible);
}

void RobotStateVisualization::setVisualVisible(bool visible)
{
  visual_visible_ = visible;
  robot_.setVisualVisible(visible);
}

void RobotStateVisualization::setCollisionVisible(bool visible)
{
  collision_visible_ = visible;
  robot_.setCollisionVisible(visible);
}

void RobotStateVisualization::setInertiaVisible(bool visible)
{
  inertia_visible_ = visible;
  robot_.setInertiaVisible(visible);
}

void RobotStateVisualization::setAlpha(double alpha)
{
  robot_.setAlpha(alpha);
}
}  // namespace tobas
