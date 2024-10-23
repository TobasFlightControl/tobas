#include <pluginlib/class_list_macros.hpp>
#include <rviz_common/properties/string_property.hpp>

#include "../include/tobas_rviz_plugin/robot_state_display.hpp"

using namespace std;

namespace tobas_rviz_plugin
{
RobotStateDisplay::RobotStateDisplay()
{
  enable_inertia_visible_ = new rviz_common::properties::BoolProperty(
    "Inertia Enabled", false, "Whether to display the inertia representation of the robot.", this,
    SLOT(changedEnableInertiaVisible()), this);

  highlight_link_ = new rviz_common::properties::StringProperty(
    "Highlight Link", "", "Highlight chosen link.", this, SLOT(changedHighlightColor()), this);

  unhighlight_link_ = new rviz_common::properties::StringProperty(
    "Unhighlight Link", "", "Unhighlight chosen link.", this, SLOT(changedUnhighlightColor()), this);

  reload_ =
    new rviz_common::properties::BoolProperty("Reload", true, "Reload robot model.", this, SLOT(changedReload()), this);
}

void RobotStateDisplay::changedEnableInertiaVisible()
{
  robot_->getRobot().setInertiaVisible(enable_inertia_visible_->getBool());
}

void RobotStateDisplay::changedHighlightColor()
{
  if (robot_ == nullptr)
  {
    RCLCPP_ERROR(node_->get_logger(), "Robot is NULL.");
    return;
  }

  std_msgs::msg::ColorRGBA color_msg;
  color_msg.r = kHighlightR;
  color_msg.g = kHighlightG;
  color_msg.b = kHighlightB;
  color_msg.a = kHighlightA;
  setHighlight(highlight_link_->getStdString(), color_msg);
  update_state_ = true;
}

void RobotStateDisplay::changedUnhighlightColor()
{
  if (robot_ == nullptr)
  {
    RCLCPP_ERROR(node_->get_logger(), "Robot is NULL.");
    return;
  }

  unsetHighlight(unhighlight_link_->getStdString());
  update_state_ = true;
}

void RobotStateDisplay::changedReload()
{
  if (reload_->getBool())
    reset();
}
}  // namespace tobas_rviz_plugin

PLUGINLIB_EXPORT_CLASS(tobas_rviz_plugin::RobotStateDisplay, rviz_common::Display)
