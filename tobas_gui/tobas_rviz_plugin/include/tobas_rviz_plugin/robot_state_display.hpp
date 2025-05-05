#pragma once

#include <rclcpp/rclcpp.hpp>
#include <rviz_common/display.hpp>
#include <rviz_common/display_context.hpp>
#include <rviz_common/frame_manager_iface.hpp>
#include <rviz_common/properties/bool_property.hpp>
#include <rviz_common/properties/color_property.hpp>
#include <rviz_common/properties/float_property.hpp>
#include <rviz_common/properties/ros_topic_property.hpp>
#include <rviz_common/properties/string_property.hpp>
#include <rviz_default_plugins/robot/robot.hpp>
#include <rviz_default_plugins/robot/robot_link.hpp>

#include <tobas_visualization_msgs/msg/display_robot_state.hpp>

#include "./rbf_loader.hpp"
#include "./robot_state_visualization.hpp"

namespace tobas
{
class RobotStateDisplay : public rviz_common::Display
{
  Q_OBJECT

  using self = RobotStateDisplay;
  using super = rviz_common::Display;

  static constexpr float kHighlightR = 0;
  static constexpr float kHighlightG = 255;
  static constexpr float kHighlightB = 0;
  static constexpr float kHighlightA = 0.7;

public:
  explicit RobotStateDisplay();

  void load(const rviz_common::Config& config) override;
  void update(float wall_dt, float ros_dt) override;
  void reset() override;

  void setLinkColor(const std::string& link_name, const QColor& color);
  void unsetLinkColor(const std::string& link_name);

public Q_SLOTS:
  void setVisible(bool visible);

private Q_SLOTS:
  void changedRobotDescription();
  void changedRootLinkName();
  void changedRobotSceneAlpha();
  void changedAttachedBodyColor();
  void changedRobotStateTopic();
  void changedEnableLinkHighlight();
  void changedEnableVisualVisible();
  void changedEnableCollisionVisible();
  void changedEnableInertiaVisible();
  void changedAllLinks();
  void changedHighlightColor();
  void changedUnhighlightColor();
  void changedReload();

protected:
  void initializeLoader();
  void loadRobotModel();

  /* Set the scene node's position, given the target frame and the planning frame. */
  void calculateOffsetPosition();

  void setLinkColor(rviz_default_plugins::robot::Robot* robot, const std::string& link_name, const QColor& color);
  void unsetLinkColor(rviz_default_plugins::robot::Robot* robot, const std::string& link_name);

  void newRobotStateCallback(const tobas_visualization_msgs::msg::DisplayRobotState::ConstSharedPtr& state);

  void
  setRobotHighlights(const tobas_visualization_msgs::msg::DisplayRobotState::_highlight_links_type& highlight_links);
  void setHighlight(const std::string& link_name, const std_msgs::msg::ColorRGBA& color);
  void unsetHighlight(const std::string& link_name);

  // overrides from Display
  void onInitialize() override;
  void onEnable() override;
  void onDisable() override;
  void fixedFrameChanged() override;

  // render the robot
  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<tobas_visualization_msgs::msg::DisplayRobotState>::SharedPtr robot_state_subscriber_;

  RobotStateVisualizationPtr robot_;
  RDFLoaderPtr rdf_loader_;
  RobotModelConstPtr robot_model_;
  RobotStatePtr robot_state_;
  std::map<std::string, std_msgs::msg::ColorRGBA> highlights_;
  bool update_state_;

  rviz_common::properties::StringProperty* robot_description_property_;
  rviz_common::properties::StringProperty* root_link_name_property_;
  rviz_common::properties::RosTopicProperty* robot_state_topic_property_;
  rviz_common::properties::FloatProperty* robot_alpha_property_;
  rviz_common::properties::ColorProperty* attached_body_color_property_;
  rviz_common::properties::BoolProperty* enable_link_highlight_;
  rviz_common::properties::BoolProperty* enable_visual_visible_;
  rviz_common::properties::BoolProperty* enable_collision_visible_;
  rviz_common::properties::BoolProperty* enable_inertia_visible_;
  rviz_common::properties::BoolProperty* show_all_links_;
  rviz_common::properties::StringProperty* highlight_link_;
  rviz_common::properties::StringProperty* unhighlight_link_;
  rviz_common::properties::BoolProperty* reload_;
};
}  // namespace tobas
