#pragma once

#include <tobas_ros2_tools/register.hpp>
#include <tobas_msgs/msg/arming.hpp>
#include <tobas_msgs_adapter/gnss.hpp>

#include "./status.hpp"

namespace gui
{
namespace gcs
{
class OtherStatusViewerWidget : public QWidget
{
  Q_OBJECT

  using self = OtherStatusViewerWidget;
  using super = QWidget;

public:
  explicit OtherStatusViewerWidget(rclcpp::Node::SharedPtr node);

  void updateNamespace(const std::string& ns);

private:
  const rclcpp::Node::SharedPtr node_;

  StatusWidget* arming_status_;
  StatusWidget* gnss_status_;

  ros2::SubscriberPtr<tobas_msgs::msg::Arming> arming_sub_;
  ros2::SubscriberPtr<tobas_msgs::Gnss> gnss_sub_;

  void armingCb(const tobas_msgs::msg::Arming::ConstSharedPtr& arming);
  void gnssCb(const tobas_msgs::Gnss::ConstSharedPtr& gnss);
};
}  // namespace gcs
}  // namespace gui
