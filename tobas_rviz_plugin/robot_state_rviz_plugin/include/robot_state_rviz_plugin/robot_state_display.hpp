#pragma once

#include <ros/ros.h>
#include <rviz/display.h>
#include <rviz/properties/property.h>
#include <rviz/properties/string_property.h>
#include <rviz/properties/bool_property.h>
#include <rviz/properties/float_property.h>
#include <rviz/properties/ros_topic_property.h>
#include <rviz/properties/color_property.h>
#include <OgreSceneNode.h>
#include <moveit/rdf_loader/rdf_loader.h>
#include <moveit/rviz_plugin_render_tools/robot_state_visualization.h>
#include <moveit_msgs/DisplayRobotState.h>

namespace moveit_rviz_plugin
{
/**
 * @brief
 * https://github.com/ros-planning/moveit/blob/master/moveit_ros/visualization/robot_state_rviz_plugin/include/moveit/robot_state_rviz_plugin/robot_state_display.h
 * Highlighting functions are added.
 */
class RobotStateDisplay : public rviz::Display
{
  Q_OBJECT

  using self = RobotStateDisplay;
  using super = rviz::Display;

public:
  explicit RobotStateDisplay();

  void update(float wall_dt, float ros_dt) override;
  void reset() override;

  void setLinkColor(const std::string& link_name, const QColor& color);
  void unsetLinkColor(const std::string& link_name);

private Q_SLOTS:
  void changedRobotDescription();
  void changedRootLinkName();
  void changedHighlightColor();
  void changedUnhighlightColor();
  void changedRobotStateTopic();
  void changedRobotSceneAlpha();
  void changedAttachedBodyColor();
  void changedEnableLinkHighlight();
  void changedEnableVisualVisible();
  void changedEnableCollisionVisible();
  void changedShowAllLinks();
  void changedReload();

protected:
  void loadRobotModel();

  /**
   * \brief Set the scene node's position, given the target frame and the planning frame
   */
  void calculateOffsetPosition();

  void setLinkColor(rviz::Robot* robot, const std::string& link_name, const QColor& color);
  void unsetLinkColor(rviz::Robot* robot, const std::string& link_name);

  void setRobotHighlights(const moveit_msgs::DisplayRobotState::_highlight_links_type& links);
  void setHighlight(const std::string& link_name, const std_msgs::ColorRGBA& color);
  void unsetHighlight(const std::string& link_name);
  void restartSubscribers();
  void robotStateCb(const moveit_msgs::DisplayRobotStateConstPtr& state);

  // overrides from Display
  void onInitialize() override;
  void onEnable() override;
  void onDisable() override;
  void fixedFrameChanged() override;

  ros::NodeHandle nh_;
  ros::Subscriber robot_state_sub_;

  RobotStateVisualizationPtr robot_;
  rdf_loader::RDFLoaderPtr rdf_loader_;
  robot_model::RobotModelConstPtr kmodel_;
  robot_state::RobotStatePtr kstate_;
  std::map<std::string, std_msgs::ColorRGBA> highlights_;
  bool update_state_ = false;
  bool load_robot_model_ = false;  // for delayed robot initialization

  std::shared_ptr<rviz::StringProperty> robot_description_property_;
  std::shared_ptr<rviz::StringProperty> root_link_name_property_;
  std::shared_ptr<rviz::StringProperty> highlight_link_;
  std::shared_ptr<rviz::StringProperty> unhighlight_link_;

  std::shared_ptr<rviz::RosTopicProperty> robot_state_topic_property_;
  std::shared_ptr<rviz::FloatProperty> robot_alpha_property_;
  std::shared_ptr<rviz::ColorProperty> attached_body_color_property_;

  std::shared_ptr<rviz::BoolProperty> enable_link_highlight_;
  std::shared_ptr<rviz::BoolProperty> enable_visual_visible_;
  std::shared_ptr<rviz::BoolProperty> enable_collision_visible_;
  std::shared_ptr<rviz::BoolProperty> show_all_links_;
  std::shared_ptr<rviz::BoolProperty> reload_;
};
}  // namespace moveit_rviz_plugin
