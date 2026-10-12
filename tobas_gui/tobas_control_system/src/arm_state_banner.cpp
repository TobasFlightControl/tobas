// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_control_system/arm_state_banner.hpp"

#include <QDebug>
#include <QFont>
#include <QFrame>
#include <QPalette>
#include <QStringList>

#include <tobas_qt_tools/color.hpp>
#include <tobas_qt_tools/font.hpp>

namespace tobas
{
namespace gui
{
namespace ctrl
{
namespace
{
void appendArmReadinessIssue(QStringList& _issues, const QString& _label, uint8_t _status)
{
  if (_status == tobas_msgs::msg::VehicleHealth::FAILED) {
    _issues << _label;
  }
  else if (_status == tobas_msgs::msg::VehicleHealth::UNKNOWN) {
    _issues << _label + " Unknown";
  }
}
}  // namespace

ArmStateBanner::ArmStateBanner(const rqt::RosQtBridge& _bridge)
{
  setAlignment(Qt::AlignCenter);
  setAutoFillBackground(true);
  setFixedHeight(30);
  setFont(qt::DefaultFont(14, QFont::Bold));
  setFrameStyle(QFrame::StyledPanel | QFrame::Sunken);

  connect(&_bridge, &rqt::RosQtBridge::armingReceived, this, &self::armingCb, Qt::QueuedConnection);
  connect(&_bridge, &rqt::RosQtBridge::vehicleHealthReceived, this, &self::healthCb, Qt::QueuedConnection);
}

void ArmStateBanner::reset()
{
  arming_.reset();
  health_.reset();

  updateState();
}

void ArmStateBanner::setConnected(bool _connected)
{
  connected_ = _connected;
  updateState();
}

QString ArmStateBanner::armReadinessIssueText() const
{
  QStringList issues;
  appendArmReadinessIssue(issues, "Realtime Compliance", health_->realtime_compliance);
  appendArmReadinessIssue(issues, "Battery Voltage", health_->battery_voltage);
  appendArmReadinessIssue(issues, "CPU Temperature", health_->cpu_temperature);
  appendArmReadinessIssue(issues, "Radio Link", health_->radio_link);
  appendArmReadinessIssue(issues, "Rotor Links", health_->rotor_links);
  appendArmReadinessIssue(issues, "Level Attitude", health_->attitude_level);
  appendArmReadinessIssue(issues, "GNSS Fix", health_->gnss_fix);
  appendArmReadinessIssue(issues, "Position Stability", health_->position_stability);
  appendArmReadinessIssue(issues, "Horizontal Position Accuracy", health_->horizontal_position_accuracy);
  appendArmReadinessIssue(issues, "Vertical Position Accuracy", health_->vertical_position_accuracy);
  appendArmReadinessIssue(issues, "Attitude Accuracy", health_->attitude_accuracy);
  appendArmReadinessIssue(issues, "Heading Accuracy", health_->heading_accuracy);
  appendArmReadinessIssue(issues, "Mag Offset", health_->mag_offset);
  appendArmReadinessIssue(issues, "Mag Alignment", health_->mag_alignment);
  appendArmReadinessIssue(issues, "Vibration Level", health_->vibration_level);
  appendArmReadinessIssue(issues, "User-Defined Condition", health_->user_defined_condition);

  if (issues.isEmpty()) {
    qWarning() << "No issues found.";
    return "";
  }

  constexpr int kMaxArmReadinessIssues = 3;
  const auto n_hidden = issues.size() - kMaxArmReadinessIssues;
  const auto visible_issues = issues.mid(0, kMaxArmReadinessIssues);
  if (n_hidden <= 0) {
    return visible_issues.join(", ");
  }

  return visible_issues.join(", ") + QString(" (+%1 more)").arg(n_hidden);
}

void ArmStateBanner::updateState()
{
  if (!connected_) {
    setStateText("Disconnected", Qt::lightGray, Qt::black);
  }
  else if (!arming_) {
    setStateText("Waiting for arming state", Qt::lightGray, Qt::black);
  }
  else if (!health_) {
    setStateText("Waiting for vehicle health", Qt::lightGray, Qt::black);
  }
  else if (arming_->data) {
    setStateText("Armed", qt::color::lightBlue(), Qt::black);
  }
  else if (health_->ok) {
    setStateText("Ready to Arm", qt::color::lightGreen(), Qt::black);
  }
  else {
    setStateText("Not Ready to Arm: " + armReadinessIssueText(), qt::color::lightYellow(), Qt::black);
  }
}

void ArmStateBanner::setStateText(const QString& _text, const QColor& _background, const QColor& _foreground)
{
  setText(_text);
  setToolTip(_text);

  auto pal = palette();
  pal.setColor(QPalette::Window, _background);
  pal.setColor(QPalette::WindowText, _foreground);
  setPalette(pal);
}

void ArmStateBanner::armingCb(const tobas_msgs::msg::Arming::ConstSharedPtr& _arming)
{
  arming_ = _arming;
  updateState();
}

void ArmStateBanner::healthCb(const tobas_msgs::msg::VehicleHealth::ConstSharedPtr& _health)
{
  health_ = _health;
  updateState();
}
}  // namespace ctrl
}  // namespace gui
}  // namespace tobas
