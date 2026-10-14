// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <tobas_qt_tools/widgets/framed_label.hpp>
#include <tobas_rqt_bridge/bridge.hpp>

namespace tobas::gui::ctrl
{
class GnssViewerWidget : public QWidget
{
  Q_OBJECT

  using self = GnssViewerWidget;

public:
  explicit GnssViewerWidget(const rqt::RosQtBridge& _bridge);

  void reset();

private:
  qt::FramedLabel* status_;
  qt::FramedLabel* nsats_;

  void setStatus(uint8_t _status);
  void setNSats(int _nsats);

private Q_SLOTS:
  void gnssCb(const tobas_msgs::Gnss::ConstSharedPtr& _gnss);
};
}  // namespace tobas::gui::ctrl
