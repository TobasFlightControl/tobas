// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_control_system/power_source_viewer/battery_viewer.hpp"

#include <algorithm>
#include <format>
#include <memory>

#include <tobas_math/core.hpp>
#include <tobas_qt_tools/color.hpp>
#include <tobas_qt_tools/layouts/form_layout.hpp>
#include <tobas_qt_tools/widgets/label.hpp>

namespace tobas::gui::ctrl
{
BatteryViewerWidget::BatteryViewerWidget(const rqt::RosQtBridge& _bridge, const Drone& _drone) : drone_(_drone)
{
  voltage_ = new qt::ProgressBar();
  current_ = new qt::ProgressBar();

  constexpr int kBarHeight = 30;
  voltage_->setFixedHeight(kBarHeight);
  current_->setFixedHeight(kBarHeight);

  // Layout
  const auto form = new qt::FormLayout(this);
  constexpr int kLabelPSize = 12;
  form->addVAlignedRow(new qt::Label("Volt", kLabelPSize), voltage_);
  form->addVAlignedRow(new qt::Label("Curr", kLabelPSize), current_);

  // Connection
  connect(&_bridge, &rqt::RosQtBridge::batteryReceived, this, &self::batteryCb, Qt::QueuedConnection);
}

void BatteryViewerWidget::reset()
{
  voltage_->reset();
  current_->reset();
}

void BatteryViewerWidget::updateInternalDataStructures()
{
  reset();

  if (drone_.prop->type() == PropulsionSystem::kElectric) {
    eprop_ = std::static_pointer_cast<ElectricPropulsionSystemConfig>(drone_.prop);
  }
  else {
    eprop_.reset();
  }
}

void BatteryViewerWidget::updateVoltage(double _voltage)
{
  const auto volt_rate = math::remap(_voltage, eprop_->battery.sag_voltage, eprop_->battery.max_voltage, 0.0, 100.0);
  voltage_->setPercentage(volt_rate);
  voltage_->setFormat(std::format("{:.2f} V ({:.0f} %)", _voltage, std::clamp(volt_rate, 0.0, 100.0)).c_str());

  if (volt_rate > 20.0) {
    voltage_->setFillColor(qt::color::green500());
  }
  else if (volt_rate > 10.0) {
    voltage_->setFillColor(qt::color::yellow500());
  }
  else {
    voltage_->setFillColor(qt::color::red500());
  }
}

void BatteryViewerWidget::updateCurrent(double _current)
{
  const auto current_rate = math::remap(_current, 0.0, eprop_->battery.max_current, 0.0, 100.0);
  current_->setPercentage(current_rate);
  current_->setFormat(std::format("{:.2f} A", _current).c_str());

  if (_current < eprop_->battery.max_current * 0.6) {
    current_->setFillColor(qt::color::green500());
  }
  else if (_current < eprop_->battery.max_current * 0.8) {
    current_->setFillColor(qt::color::yellow500());
  }
  else {
    current_->setFillColor(qt::color::red500());
  }
}

void BatteryViewerWidget::batteryCb(const tobas_msgs::msg::Battery::ConstSharedPtr& _battery)
{
  if (!eprop_) {
    return;
  }

  updateVoltage(_battery->voltage);
  updateCurrent(_battery->current);
}
}  // namespace tobas::gui::ctrl
