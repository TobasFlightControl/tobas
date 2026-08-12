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
void appendArmReadinessIssue(QStringList& issues, const QString& label, uint8_t status)
{
  if (status == tobas_msgs::msg::VehicleHealth::FAILED) {
    issues << label;
  }
  else if (status == tobas_msgs::msg::VehicleHealth::UNKNOWN) {
    issues << label + " Unknown";
  }
}
}  // namespace

ArmStateBanner::ArmStateBanner(const RosQtBridge& bridge)
{
  setAlignment(Qt::AlignCenter);
  setAutoFillBackground(true);
  setFixedHeight(kHeight);
  setFont(qt::DefaultFont(kPSize, QFont::Bold));
  setFrameStyle(QFrame::StyledPanel | QFrame::Sunken);

  connect(&bridge, &RosQtBridge::armingReceived, this, &self::armingCb, Qt::QueuedConnection);
  connect(&bridge, &RosQtBridge::vehicleHealthReceived, this, &self::healthCb, Qt::QueuedConnection);
}

void ArmStateBanner::reset()
{
  arming_.reset();
  health_.reset();

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

  const auto n_hidden = issues.size() - kMaxArmReadinessIssues;
  const auto visible_issues = issues.mid(0, kMaxArmReadinessIssues);
  if (n_hidden <= 0) {
    return visible_issues.join(", ");
  }

  return visible_issues.join(", ") + QString(" (+%1 more)").arg(n_hidden);
}

void ArmStateBanner::updateState()
{
  if (!arming_) {
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

void ArmStateBanner::setStateText(const QString& text, const QColor& background, const QColor& foreground)
{
  setText(text);
  setToolTip(text);

  auto pal = palette();
  pal.setColor(QPalette::Window, background);
  pal.setColor(QPalette::WindowText, foreground);
  setPalette(pal);
}

void ArmStateBanner::armingCb(const tobas_msgs::msg::Arming::ConstSharedPtr& arming)
{
  arming_ = arming;
  updateState();
}

void ArmStateBanner::healthCb(const tobas_msgs::msg::VehicleHealth::ConstSharedPtr& health)
{
  health_ = health;
  updateState();
}
}  // namespace ctrl
}  // namespace gui
}  // namespace tobas
