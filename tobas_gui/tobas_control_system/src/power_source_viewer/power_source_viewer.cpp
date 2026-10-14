// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_control_system/power_source_viewer/power_source_viewer.hpp"

namespace tobas::gui::ctrl
{
PowerSourceViewerWidget::PowerSourceViewerWidget(const rqt::RosQtBridge& _bridge, const Drone& _drone) : drone_(_drone)
{
  battery_viewer_ = new BatteryViewerWidget(_bridge, _drone);
  engine_viewer_ = new EngineViewerWidget(_bridge, _drone);

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

  // Switch displayed widgets according to the propulsion system.
  switch (drone_.prop->type()) {
    case PropulsionSystem::kElectric:
      battery_viewer_->updateInternalDataStructures();
      setCurrentWidget(battery_viewer_);
      break;
    case PropulsionSystem::kIce:
      engine_viewer_->updateInternalDataStructures();
      setCurrentWidget(engine_viewer_);
      break;
    default:
      throw;
  }
}
}  // namespace tobas::gui::ctrl
