// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <QColor>
#include <QLabel>
#include <QString>
#include <QWidget>

#include <tobas_constants/flight_mode.hpp>
#include <tobas_rqt_bridge/bridge.hpp>

#include "./stick_panel.hpp"

namespace tobas
{
namespace gui
{
namespace ctrl
{
namespace rcin
{
class RCInputViewerWidget : public QWidget
{
  Q_OBJECT

  using self = RCInputViewerWidget;
  using super = QWidget;

public:
  explicit RCInputViewerWidget(const RosQtBridge& bridge);

  void reset();

private:
  StickPanel* yaw_pitch_;
  StickPanel* roll_throttle_;

  QLabel* enable_;
  QLabel* kill_;
  QLabel* sub_mode_;
  QLabel* acrobat_;
  QLabel* stabilize_;
  QLabel* loiter_;

  QLabel* makeBadge(const QString& text) const;
  void setBadge(QLabel* badge, const QString& text, const QColor& background);
  void setMode(FlightMode mode);

private Q_SLOTS:
  void rcInputCb(const tobas_msgs::RCInput::ConstSharedPtr& rcin);
};
}  // namespace rcin
}  // namespace ctrl
}  // namespace gui
}  // namespace tobas
