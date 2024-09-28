#pragma once

#include <tobas_ros2_tools/register.hpp>
#include <tobas_drone_core/drone.hpp>
#include <tobas_qt_tools/widgets/scroll_area.hpp>

#include "./rotor.hpp"

namespace gui
{
namespace control_system
{
class RotorsViewerWiddget : public qt::ScrollArea
{
  Q_OBJECT

  using self = RotorsViewerWiddget;
  using super = qt::ScrollArea;

public:
  explicit RotorsViewerWiddget(rclcpp::Node::SharedPtr node, const tobas::Drone& drone);

  void updateInternalDataStructures();

private:
  const rclcpp::Node::SharedPtr node_;
  const tobas::Drone& drone_;
};
}  // namespace control_system
}  // namespace gui
