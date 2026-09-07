// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_control_system/power_source_viewer/engine_viewer.hpp"

#include <algorithm>
#include <format>

#include <boost/polymorphic_pointer_cast.hpp>

#include <tobas_math/core.hpp>
#include <tobas_qt_tools/color.hpp>
#include <tobas_qt_tools/layouts/form_layout.hpp>
#include <tobas_qt_tools/message.hpp>
#include <tobas_qt_tools/widgets/label.hpp>

namespace tobas
{
namespace gui
{
namespace ctrl
{
EngineViewerWidget::EngineViewerWidget(const rqt::RosQtBridge& bridge, const Drone& drone) : drone_(drone)
{
  fuel_quantity_ = new qt::ProgressBar();
  oil_temp_ = new qt::ProgressBar();

  constexpr int kBarHeight = 30;
  fuel_quantity_->setFixedHeight(kBarHeight);
  oil_temp_->setFixedHeight(kBarHeight);

  // Layout
  const auto form = new qt::FormLayout();
  constexpr int kLabelPSize = 12;
  form->addVAlignedRow(new qt::Label("Fuel QTY", kLabelPSize), fuel_quantity_);
  form->addVAlignedRow(new qt::Label("Oil Temp", kLabelPSize), oil_temp_);
  setLayout(form);

  // Connection
  connect(&bridge, &rqt::RosQtBridge::engineStateReceived, this, &self::engineStateCb, Qt::QueuedConnection);
}

void EngineViewerWidget::reset()
{
  fuel_quantity_->reset();
  oil_temp_->reset();
}

void EngineViewerWidget::updateInternalDataStructures()
{
  reset();

  if (drone_.prop->type() == PropulsionSystem::kIce) {
    iprop_ = boost::polymorphic_pointer_downcast<IcePropulsionSystemConfig>(drone_.prop);
  }
  else {
    iprop_.reset();
  }
}

void EngineViewerWidget::updateFuelQuantity(const double& fuel_quantity)
{
  constexpr double kMaxFuelQuantity = 100.0;  // TODO: Include fuel capacity in `EngineConfig`.
  const auto fuel_rate = math::remap(fuel_quantity, 0.0, kMaxFuelQuantity, 0.0, 100.0);
  fuel_quantity_->setPercentage(fuel_rate);
  fuel_quantity_->setFormat(
    std::format("{:.2f} L ({:.0f} %)", fuel_quantity, std::clamp(fuel_rate, 0.0, 100.0)).c_str());

  if (fuel_rate > 20.0) {
    fuel_quantity_->setFillColor(qt::color::green500());
  }
  else if (fuel_rate > 10.0) {
    fuel_quantity_->setFillColor(qt::color::yellow500());
  }
  else {
    fuel_quantity_->setFillColor(qt::color::red500());
  }
}

void EngineViewerWidget::updateOilTemperature(const double& oil_temp)
{
  constexpr double kMinOilTemp = 0.0;    // [degC]
  constexpr double kMaxOilTemp = 130.0;  // [degC]
  const auto oil_temp_rate = math::remap(oil_temp, kMinOilTemp, kMaxOilTemp, 0.0, 100.0);
  oil_temp_->setPercentage(oil_temp_rate);
  oil_temp_->setFormat(std::format("{:.1f} ℃", oil_temp).c_str());

  // TODO: Include the proper oil temperature range in `EngineConfig`.
  if (oil_temp < 60.0) {
    oil_temp_->setFillColor(qt::color::cyan500());
  }
  else if (oil_temp < 100.0) {
    oil_temp_->setFillColor(qt::color::green500());
  }
  else if (oil_temp < 120.0) {
    oil_temp_->setFillColor(qt::color::yellow500());
  }
  else {
    oil_temp_->setFillColor(qt::color::red500());
  }
}

void EngineViewerWidget::engineStateCb(const tobas_msgs::msg::EngineState::ConstSharedPtr& engine_state)
{
  if (!iprop_) {
    return;
  }

  updateFuelQuantity(engine_state->fuel_quantity);
  updateOilTemperature(engine_state->oil_temperature);
}
}  // namespace ctrl
}  // namespace gui
}  // namespace tobas
