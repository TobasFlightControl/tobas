#include <rviz_common/properties/string_property.hpp>
#include <class_loader/class_loader.hpp>

#include "../include/tobas_rviz_plugin/robot_state_display.hpp"

using namespace std;

namespace tobas_rviz_plugin
{
RobotStateDisplay::RobotStateDisplay()
{
  highlight_link_ = new rviz_common::properties::StringProperty(
    "Highlight Link", "", "Highlight chosen link.", this, SLOT(changedHighlightColor()), this);

  unhighlight_link_ = new rviz_common::properties::StringProperty(
    "Unhighlight Link", "", "Unhighlight chosen link.", this, SLOT(changedUnhighlightColor()), this);
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
}  // namespace tobas_rviz_plugin

CLASS_LOADER_REGISTER_CLASS(tobas_rviz_plugin::RobotStateDisplay, rviz_common::Display)
