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

  static constexpr int kLabelPSize = 12;
  static constexpr int kBarHeight = 30;

  static constexpr double kMinTemp = 0.;    // [degC]
  static constexpr double kMaxTemp = 100.;  // [degC]

public:
  explicit CpuViewerWidget(const RosQtBridge& bridge);

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
