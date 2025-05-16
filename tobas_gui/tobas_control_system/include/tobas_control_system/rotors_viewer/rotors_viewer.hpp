#pragma once

#include <QHBoxLayout>

#include <tobas_drone_core/drone.hpp>
#include <tobas_qt_tools/widgets/scroll_area.hpp>
#include <tobas_ros2_tools/register.hpp>

#include <tobas_msgs/msg/rotor_liveliness_array.hpp>
#include <tobas_msgs/msg/rotor_state_array.hpp>

#include "./speedmeter.hpp"

namespace gui
{
namespace gcs
{
class RotorsViewerWiddget : public qt::ScrollArea
{
  Q_OBJECT

  using self = RotorsViewerWiddget;
  using super = qt::ScrollArea;

  static constexpr char kAliveBackgroundColor[] = "transparent";
  static constexpr char kDeadBackgroundColor[] = "red";

public:
  explicit RotorsViewerWiddget(rclcpp::Node::SharedPtr node, const tobas::Drone& drone);

  void reset();
  void updateInternalDataStructures();

private:
  const rclcpp::Node::SharedPtr node_;
  const tobas::Drone& drone_;

  std::map<std::string, SpeedmeterWidget*> meters_;
  QHBoxLayout* cols_;

  ros2::SubscriberPtr<tobas_msgs::msg::RotorStateArray> rotor_states_sub_;
  ros2::SubscriberPtr<tobas_msgs::msg::RotorLivelinessArray> rotor_liveliness_sub_;

  void setSpeed(const std::string& link_name, const double& rps);

  void rotorStatesCb(const tobas_msgs::msg::RotorStateArray::ConstSharedPtr& msg);
  void rotorLivelinessCb(const tobas_msgs::msg::RotorLivelinessArray::ConstSharedPtr& msg);

  static QString bottomText(int rpm);
};
}  // namespace gcs
}  // namespace gui
