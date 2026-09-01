// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_control_system/cpu_viewer.hpp"

#include <format>

#include <tobas_math/core.hpp>
#include <tobas_qt_tools/color.hpp>
#include <tobas_qt_tools/layouts/form_layout.hpp>
#include <tobas_qt_tools/widgets/label.hpp>

namespace tobas
{
namespace gui
{
namespace ctrl
{
namespace
{
constexpr int kLabelPSize = 12;
constexpr int kBarHeight = 30;

constexpr double kMinTemp = 0.0;    // [degC]
constexpr double kMaxTemp = 100.0;  // [degC]
}  // namespace

CpuViewerWidget::CpuViewerWidget(const rqt::RosQtBridge& bridge)
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
  connect(&bridge, &rqt::RosQtBridge::cpuReceived, this, &self::cpuCb, Qt::QueuedConnection);
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
