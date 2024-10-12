#pragma once

#include <tobas_drone_core/drone.hpp>

#include "./pose_viewer.hpp"
#include "./battery_cpu_viewer.hpp"
#include "./rcin_viewer/rcin_viewer.hpp"
#include "./rotors_viewer/rotors_viewer.hpp"
#include "./console.hpp"
#include "./status_viewer/status_viewer.hpp"
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
  BatteryCPUViewerWidget* battery_cpu_viewer_;
  rcin::RCInputViewerWidget* rcin_viewer_;
  RotorsViewerWiddget* rotors_viewer_;
  ConsoleWidget* console_;
  StatusViewerWidget* status_viewer_;
  MissionPlannerWidget* mission_planner_;
};
}  // namespace control_system
}  // namespace gui
