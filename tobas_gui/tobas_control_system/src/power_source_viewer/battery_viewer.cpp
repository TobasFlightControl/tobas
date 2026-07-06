// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_control_system/power_source_viewer/battery_viewer.hpp"

#include <algorithm>
#include <format>

#include <boost/polymorphic_pointer_cast.hpp>

#include <tobas_math/core.hpp>
#include <tobas_qt_tools/layouts/form_layout.hpp>
#include <tobas_qt_tools/message.hpp>
#include <tobas_qt_tools/widgets/label.hpp>

namespace tobas
{
namespace gui
{
namespace ctrl
{
BatteryViewerWidget::BatteryViewerWidget(const RosQtBridge& bridge, const Drone& drone) : drone_(drone)
{
  voltage_ = new qt::ProgressBar();
  current_ = new qt::ProgressBar();

  voltage_->setFixedHeight(kBarHeight);
  current_->setFixedHeight(kBarHeight);

  // Layout
  const auto form = new qt::FormLayout();
  form->addVAlignedRow(new qt::Label("Volt", kLabelPSize), voltage_);
  form->addVAlignedRow(new qt::Label("Curr", kLabelPSize), current_);
  setLayout(form);

  // Connection
  connect(&bridge, &RosQtBridge::batteryReceived, this, &self::batteryCb, Qt::QueuedConnection);
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
    eprop_ = boost::polymorphic_pointer_downcast<ElectricPropulsionSystemConfig>(drone_.prop);
  }
  else {
    eprop_.reset();
  }
}

void BatteryViewerWidget::updateVoltage(const double& voltage)
{
  const auto volt_rate = math::remap(voltage, eprop_->battery.sag_voltage, eprop_->battery.max_voltage, 0.0, 100.0);
  voltage_->setPercentage(volt_rate);
  voltage_->setFormat(std::format("{:.2f} V ({:.0f} %)", voltage, std::clamp(volt_rate, 0.0, 100.0)).c_str());

  if (volt_rate > 20.0) {
    voltage_->setFillColor(Qt::green);
  }
  else if (volt_rate > 10.0) {
    voltage_->setFillColor(Qt::yellow);
  }
  else {
    voltage_->setFillColor(Qt::red);
  }
}

void BatteryViewerWidget::updateCurrent(const double& current)
{
  const auto current_rate = math::remap(current, 0.0, eprop_->battery.max_current, 0.0, 100.0);
  current_->setPercentage(current_rate);
  current_->setFormat(std::format("{:.2f} A", current).c_str());

  if (current < eprop_->battery.max_current * 0.6) {
    current_->setFillColor(Qt::green);
  }
  else if (current < eprop_->battery.max_current * 0.8) {
    current_->setFillColor(Qt::yellow);
  }
  else {
    current_->setFillColor(Qt::red);
  }
}

void BatteryViewerWidget::batteryCb(const tobas_msgs::msg::Battery::ConstSharedPtr& battery)
{
  if (!eprop_) {
    return;
  }

  updateVoltage(battery->voltage);
  updateCurrent(battery->current);
}
}  // namespace ctrl
}  // namespace gui
}  // namespace tobas
