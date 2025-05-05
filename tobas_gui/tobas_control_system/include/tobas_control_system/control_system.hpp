#pragma once

#include "./console.hpp"
#include "./cpu_viewer.hpp"
#include "./mission_planner/mission_planner.hpp"
#include "./pose_viewer.hpp"
#include "./power_source_viewer/power_source_viewer.hpp"
#include "./rcin_viewer/rcin_viewer.hpp"
#include "./rotors_viewer/rotors_viewer.hpp"
#include "./status_viewer/status_viewer.hpp"

namespace gui
{
namespace gcs
{
class ControlSystemWidget : public QWidget
{
  Q_OBJECT

public:
  explicit ControlSystemWidget(rclcpp::Node::SharedPtr node, const tobas::Drone& drone);

  void reset();
  void updateInternalDataStructures();

private:
  const tobas::Drone& drone_;

  PoseViewerWidget* pose_viewer_;
  PowerSourceViewerWidget* power_source_viewer_;
  CPUViewerWidget* cpu_viewer_;
  rcin::RCInputViewerWidget* rcin_viewer_;
  RotorsViewerWiddget* rotors_viewer_;
  ConsoleWidget* console_;
  StatusViewerWidget* status_viewer_;
  MissionPlannerWidget* mission_planner_;
};
}  // namespace gcs
}  // namespace gui
