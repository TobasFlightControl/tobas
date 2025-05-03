#include <QVBoxLayout>

#include "tobas_control_system/power_source_viewer/power_source_viewer.hpp"

namespace gui
{
namespace gcs
{
PowerSourceViewerWidget::PowerSourceViewerWidget(rclcpp::Node::SharedPtr node, const tobas::Drone& drone)
  : drone_(drone)
{
  battery_viewer_ = new BatteryViewerWidget(node, drone);
  engine_viewer_ = new EngineViewerWidget(node, drone);

  addWidget(battery_viewer_);
  addWidget(engine_viewer_);
}

void PowerSourceViewerWidget::reset()
{
  battery_viewer_->reset();
  engine_viewer_->reset();
}

void PowerSourceViewerWidget::updateInternalDataStructures()
{
  if (!drone_.prop) {
    return;
  }

  // 推進系によって表示するウィジェットを切り替える
  switch (drone_.prop->type()) {
    case tobas::propulsion_system_t::ELECTRIC:
      battery_viewer_->updateInternalDataStructures();
      setCurrentWidget(battery_viewer_);
      break;
    case tobas::propulsion_system_t::ICE:
      engine_viewer_->updateInternalDataStructures();
      setCurrentWidget(engine_viewer_);
      break;
    default:
      throw;
  }
}
}  // namespace gcs
}  // namespace gui
