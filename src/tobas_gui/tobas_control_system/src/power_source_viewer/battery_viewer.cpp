// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_control_system/power_source_viewer/battery_viewer.hpp"

#include <algorithm>
#include <memory>

#include <QFormLayout>
#include <QString>

#include <tobas_math/core.hpp>
#include <tobas_qt_tools/color.hpp>

namespace tobas::gui::ctrl
{
BatteryViewerWidget::BatteryViewerWidget(const rqt::RosQtBridge& _bridge, const Drone& _drone) : drone_(_drone)
{
  voltage_ = new qt::ProgressBar();
  current_ = new qt::ProgressBar();

  // Layout
  const auto form = new QFormLayout(this);
  form->setContentsMargins(2, 0, 2, 0);
  form->setSpacing(2);
  form->setLabelAlignment(Qt::AlignLeft);
  form->addRow("Voltage", voltage_);
  form->addRow("Current", current_);

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
  voltage_->setFormat(QString("%1 V (%2 %)").arg(_voltage, 0, 'f', 1).arg(std::clamp(volt_rate, 0.0, 100.0), 0, 'f', 0));

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
  current_->setFormat(QString("%1 A").arg(_current, 0, 'f', 2));

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
