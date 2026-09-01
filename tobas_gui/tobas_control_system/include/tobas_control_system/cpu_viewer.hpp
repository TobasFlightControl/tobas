// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <QWidget>

#include <tobas_qt_tools/widgets/progress_bar.hpp>
#include <tobas_rqt_bridge/bridge.hpp>

namespace tobas
{
namespace gui
{
namespace ctrl
{
class CpuViewerWidget : public QWidget
{
  Q_OBJECT

  using self = CpuViewerWidget;
  using super = QWidget;

public:
  explicit CpuViewerWidget(const rqt::RosQtBridge& bridge);

  void reset();

private:
  qt::ProgressBar* temp_;
  qt::ProgressBar* load_;

private Q_SLOTS:
  void cpuCb(const tobas_msgs::msg::Cpu::ConstSharedPtr& cpu);
};
}  // namespace ctrl
}  // namespace gui
}  // namespace tobas
