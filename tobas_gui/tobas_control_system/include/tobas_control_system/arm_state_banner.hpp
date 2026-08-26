// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <QColor>
#include <QLabel>
#include <QString>

#include <tobas_rqt_bridge/bridge.hpp>

namespace tobas
{
namespace gui
{
namespace ctrl
{
class ArmStateBanner : public QLabel
{
  Q_OBJECT

  using self = ArmStateBanner;
  using super = QLabel;

  static constexpr int kHeight = 30;
  static constexpr int kPSize = 14;
  static constexpr int kMaxArmReadinessIssues = 3;

public:
  explicit ArmStateBanner(const rqt::RosQtBridge& bridge);

  void reset();

private:
  tobas_msgs::msg::Arming::ConstSharedPtr arming_;
  tobas_msgs::msg::VehicleHealth::ConstSharedPtr health_;

  QString armReadinessIssueText() const;
  void updateState();
  void setStateText(const QString& text, const QColor& background, const QColor& foreground);

private Q_SLOTS:
  void armingCb(const tobas_msgs::msg::Arming::ConstSharedPtr& arming);
  void healthCb(const tobas_msgs::msg::VehicleHealth::ConstSharedPtr& health);
};
}  // namespace ctrl
}  // namespace gui
}  // namespace tobas
