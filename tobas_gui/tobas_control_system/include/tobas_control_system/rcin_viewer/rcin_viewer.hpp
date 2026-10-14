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

namespace tobas::gui::ctrl::rcin
{
class RCInputViewerWidget : public QWidget
{
  Q_OBJECT

  using self = RCInputViewerWidget;

public:
  explicit RCInputViewerWidget(const rqt::RosQtBridge& _bridge);

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

  QLabel* makeBadge(const QString& _text) const;
  void setBadge(QLabel* _badge, const QString& _text, const QColor& _background);
  void setMode(FlightMode _mode);

private Q_SLOTS:
  void rcInputCb(const tobas_msgs::RCInput::ConstSharedPtr& _rcin);
};
}  // namespace tobas::gui::ctrl::rcin
