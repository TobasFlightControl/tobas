// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_control_system/cpu_viewer.hpp"

#include <format>

#include <tobas_qt_tools/layouts/form_layout.hpp>
#include <tobas_qt_tools/widgets/label.hpp>

namespace tobas
{
namespace gui
{
namespace ctrl
{
CpuViewerWidget::CpuViewerWidget(const RosQtBridge& bridge)
{
  temp_ = new qt::ProgressBar();
  temp_->setFixedHeight(kBarHeight);

  load_ = new qt::ProgressBar();
  load_->setFixedHeight(kBarHeight);

  // Layout
  const auto form = new qt::FormLayout();
  form->addVAlignedRow(new qt::Label("Temp", kLabelPSize), temp_);
  form->addVAlignedRow(new qt::Label("Load", kLabelPSize), load_);
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
    temp_->setFillColor(Qt::magenta);
  }
  else if (cpu->temperature > 80.0) {
    temp_->setFillColor(Qt::red);
  }
  else if (cpu->temperature > 60.0) {
    temp_->setFillColor(Qt::yellow);
  }
  else if (cpu->temperature > 0.0) {
    temp_->setFillColor(Qt::green);
  }
  else {
    temp_->setFillColor(Qt::blue);
  }

  const auto load_percent = cpu->load * 100.0;
  load_->setPercentage(load_percent);
  load_->setFormat(std::format("{:.0f} %", load_percent).c_str());
  if (load_percent > 80.0) {
    load_->setFillColor(Qt::red);
  }
  else if (load_percent > 60.0) {
    load_->setFillColor(Qt::yellow);
  }
  else {
    load_->setFillColor(Qt::green);
  }
}
}  // namespace ctrl
}  // namespace gui
}  // namespace tobas
