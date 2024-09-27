#pragma once

#include <tobas_drone_core/drone.hpp>

#include "./pose_viewer.hpp"
#include "./mission_planner/mission_planner.hpp"

namespace gui
{
namespace control_system
{
class ControlSystemWidget : public QWidget
{
  Q_OBJECT

public:
  explicit ControlSystemWidget(rclcpp::Node::SharedPtr node, const tobas::Drone& drone);

  void updateInternalDataStructures();

private:
  const rclcpp::Node::SharedPtr node_;
  const tobas::Drone& drone_;

  PoseViewerWidget* pose_viewer_;
  MissionPlannerWidget* mission_planner_;
};
}  // namespace control_system
}  // namespace gui
