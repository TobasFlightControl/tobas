// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_control_system/cpu_viewer.hpp"

#include <format>

#include <QFormLayout>

#include <tobas_qt_tools/color.hpp>

namespace tobas
{
namespace gui
{
namespace ctrl
{
CpuViewerWidget::CpuViewerWidget(const RosQtBridge& bridge)
{
  temp_ = new qt::ProgressBar();

  load_ = new qt::ProgressBar();

  // Layout
  const auto form = new QFormLayout();
  form->setContentsMargins(2, 0, 2, 0);
  form->setSpacing(2);
  form->setLabelAlignment(Qt::AlignLeft | Qt::AlignVCenter);
  form->addRow("CPU Temp", temp_);
  form->addRow("CPU Load", load_);
  setLayout(form);

  // Connection
  connect(&bridge, &RosQtBridge::cpuReceived, this, &self::cpuCb, Qt::QueuedConnection);
}

void CpuViewerWidget::reset()
{
  temp_->reset();
  load_->reset();
}

void CpuViewerWidget::cpuCb(const tobas_msgs::msg::Cpu::ConstSharedPtr& cpu)
{
  const auto temp_rate = math::remap(cpu->temperature, kMinTemp, kMaxTemp, 0.0, 100.0);
  temp_->setPercentage(temp_rate);
  temp_->setFormat(std::format("{:.0f} ℃", cpu->temperature).c_str());
  if (cpu->temperature > 85.0) {
    temp_->setFillColor(qt::color::magenta500());
  }
  else if (cpu->temperature > 80.0) {
    temp_->setFillColor(qt::color::red500());
  }
  else if (cpu->temperature > 60.0) {
    temp_->setFillColor(qt::color::yellow500());
  }
  else if (cpu->temperature > 0.0) {
    temp_->setFillColor(qt::color::green500());
  }
  else {
    temp_->setFillColor(qt::color::cyan500());
  }

  const auto load_percent = cpu->load * 100.0;
  load_->setPercentage(load_percent);
  load_->setFormat(std::format("{:.0f} %", load_percent).c_str());
  if (load_percent > 80.0) {
    load_->setFillColor(qt::color::red500());
  }
  else if (load_percent > 60.0) {
    load_->setFillColor(qt::color::yellow500());
  }
  else {
    load_->setFillColor(qt::color::green500());
  }
}
}  // namespace ctrl
}  // namespace gui
}  // namespace tobas
