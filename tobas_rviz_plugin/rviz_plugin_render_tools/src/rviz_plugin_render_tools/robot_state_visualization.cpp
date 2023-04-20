#include <QApplication>

#include "../../include/rviz_plugin_render_tools/planning_link_updater.hpp"
#include "../../include/rviz_plugin_render_tools/robot_state_visualization.hpp"

using namespace std;

namespace moveit_rviz_plugin
{
RobotStateVisualization::RobotStateVisualization(
  Ogre::SceneNode* root_node,
  rviz::DisplayContext* context,
  const string& name,
  rviz::Property* parent_property)
  : robot_(root_node, context, name, parent_property),
    visible_(true),
    visual_visible_(true),
    collision_visible_(false)
{
  default_attached_object_color_.r = 0.0f;
  default_attached_object_color_.g = 0.7f;
  default_attached_object_color_.b = 0.0f;
  default_attached_object_color_.a = 1.0f;
}

void RobotStateVisualization::load(const urdf::ModelInterface& descr, bool visual, bool collision)
{
  // clear previously loaded model
  clear();

  robot_.load(descr, visual, collision);
  robot_.setVisualVisible(visual_visible_);
  robot_.setCollisionVisible(collision_visible_);
  robot_.setVisible(visible_);
  QApplication::processEvents();
}

void RobotStateVisualization::clear()
{
  robot_.clear();
}

void RobotStateVisualization::setDefaultAttachedObjectColor(
  const std_msgs::ColorRGBA& default_attached_object_color)
{
  default_attached_object_color_ = default_attached_object_color;
}

void RobotStateVisualization::update(const robot_state::RobotStateConstPtr& kinematic_state)
{
  updateHelper(kinematic_state, default_attached_object_color_, NULL);
}

void RobotStateVisualization::update(
  const robot_state::RobotStateConstPtr& kinematic_state,
  const std_msgs::ColorRGBA& default_attached_object_color)
{
  updateHelper(kinematic_state, default_attached_object_color, NULL);
}

void RobotStateVisualization::update(
  const robot_state::RobotStateConstPtr& kinematic_state,
  const std_msgs::ColorRGBA& default_attached_object_color,
  const map<string, std_msgs::ColorRGBA>& color_map)
{
  updateHelper(kinematic_state, default_attached_object_color, &color_map);
}

void RobotStateVisualization::updateHelper(
  const robot_state::RobotStateConstPtr& kinematic_state,
  const std_msgs::ColorRGBA& default_attached_object_color,
  const map<string, std_msgs::ColorRGBA>* color_map)
{
  robot_.update(PlanningLinkUpdater(kinematic_state));

  vector<const robot_state::AttachedBody*> attached_bodies;
  kinematic_state->getAttachedBodies(attached_bodies);

  robot_.setVisualVisible(visual_visible_);
  robot_.setCollisionVisible(collision_visible_);
  robot_.setVisible(visible_);
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

void RobotStateVisualization::setAlpha(float alpha)
{
  robot_.setAlpha(alpha);
}
}  // namespace moveit_rviz_plugin
