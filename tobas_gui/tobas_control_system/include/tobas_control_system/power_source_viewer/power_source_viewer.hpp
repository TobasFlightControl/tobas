#pragma once

#include <tobas_qt_tools/widgets/stacked_widget.hpp>

#include "./battery_viewer.hpp"
#include "./engine_viewer.hpp"

namespace tobas
{
namespace gui
{
namespace ctrl
{
class PowerSourceViewerWidget : public qt::StackedWidget
{
  Q_OBJECT

public:
  explicit PowerSourceViewerWidget(const RosQtBridge& bridge, const Drone& drone);

  void reset();
  void updateInternalDataStructures();

private:
  const Drone& drone_;

  BatteryViewerWidget* battery_viewer_;
  EngineViewerWidget* engine_viewer_;
};
}  // namespace ctrl
}  // namespace gui
}  // namespace tobas
