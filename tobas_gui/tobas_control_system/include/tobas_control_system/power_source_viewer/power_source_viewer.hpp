#pragma once

#include <tobas_qt_tools/widgets/stacked_widget.hpp>

#include "./battery_viewer.hpp"
#include "./engine_viewer.hpp"

namespace gui
{
namespace gcs
{
class PowerSourceViewerWidget : public qt::StackedWidget
{
  Q_OBJECT

public:
  explicit PowerSourceViewerWidget(rclcpp::Node::SharedPtr node, const tobas::Drone& drone);

  void reset();
  void updateInternalDataStructures();

private:
  const tobas::Drone& drone_;

  BatteryViewerWidget* battery_viewer_;
  EngineViewerWidget* engine_viewer_;
};
}  // namespace gcs
}  // namespace gui
