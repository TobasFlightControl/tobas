// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_control_system/rcin_viewer/rcin_viewer.hpp"

#include <QFrame>
#include <QHBoxLayout>
#include <QPalette>
#include <QVBoxLayout>

#include <tobas_qt_tools/util.hpp>

namespace tobas
{
namespace gui
{
namespace ctrl
{
namespace rcin
{
RCInputViewerWidget::RCInputViewerWidget(const RosQtBridge& bridge)
{
  roll_pitch_ = new StickPanel("Roll / Pitch", "Roll", "Pitch");
  yaw_throttle_ = new StickPanel("Yaw / Throttle", "Yaw", "Throttle");

  enable_ = makeBadge("---");
  kill_ = makeBadge("Kill: ---");
  sub_mode_ = makeBadge("Sub: ---");
  acrobat_ = makeBadge("Acrobat");
  stabilize_ = makeBadge("Stabilize");
  loiter_ = makeBadge("Loiter");

  // Layout
  const auto switch_cols = new QHBoxLayout();
  switch_cols->addWidget(kill_, 1);
  switch_cols->addWidget(sub_mode_, 1);

  const auto mode_rows = new QVBoxLayout();
  qt::addWidgetCenter(new QLabel("Flight Mode"), mode_rows);
  mode_rows->addWidget(acrobat_);
  mode_rows->addWidget(stabilize_);
  mode_rows->addWidget(loiter_);

  const auto status_rows = new QVBoxLayout();
  status_rows->addWidget(enable_);
  status_rows->addLayout(switch_cols);
  status_rows->addStretch();
  status_rows->addLayout(mode_rows);

  const auto cols = new QHBoxLayout();
  cols->addWidget(roll_pitch_, 1);
  cols->addWidget(yaw_throttle_, 1);
  cols->addLayout(status_rows);
  setLayout(cols);

  // Connection
  connect(&bridge, &RosQtBridge::rcInputReceived, this, &self::rcInputCb, Qt::QueuedConnection);
}

void RCInputViewerWidget::reset()
{
  roll_pitch_->reset();
  yaw_throttle_->reset();
  setBadge(enable_, "---", QColor(240, 240, 240));
  setBadge(kill_, "Kill: ---", QColor(238, 238, 238));
  setBadge(sub_mode_, "Sub: ---", QColor(238, 238, 238));
  setMode(static_cast<FlightMode>(-1));
}

QLabel* RCInputViewerWidget::makeBadge(const QString& text) const
{
  const auto badge = new QLabel(text);
  badge->setAlignment(Qt::AlignCenter);
  badge->setAutoFillBackground(true);
  badge->setFrameStyle(QFrame::StyledPanel | QFrame::Sunken);
  badge->setMinimumHeight(30);
  badge->setMinimumWidth(70);
  return badge;
}

void RCInputViewerWidget::setBadge(QLabel* badge, const QString& text, const QColor& background)
{
  badge->setText(text);

  auto pal = badge->palette();
  pal.setColor(QPalette::Window, background);
  pal.setColor(QPalette::WindowText, palette().color(QPalette::WindowText));
  badge->setPalette(pal);
}

void RCInputViewerWidget::setMode(FlightMode mode)
{
  setBadge(acrobat_, "Acrobat", mode == FlightMode::kAcrobat ? QColor(220, 236, 255) : QColor(238, 238, 238));
  setBadge(stabilize_, "Stabilize", mode == FlightMode::kStabilize ? QColor(220, 236, 255) : QColor(238, 238, 238));
  setBadge(loiter_, "Loiter", mode == FlightMode::kLoiter ? QColor(220, 236, 255) : QColor(238, 238, 238));
}

void RCInputViewerWidget::rcInputCb(const tobas_msgs::RCInput::ConstSharedPtr& rcin)
{
  if (!rcin->ok) {
    reset();
    return;
  }

  roll_pitch_->setValues(rcin->roll, rcin->pitch, true, rcin->enable);
  yaw_throttle_->setValues(-rcin->yaw, rcin->throttle, true, rcin->enable);
  setBadge(enable_, rcin->enable ? "Enabled" : "Disabled", rcin->enable ? QColor(220, 236, 255) : QColor(238, 238, 238));
  setBadge(
    kill_, QString("Kill: ") + (rcin->kill ? "ON" : "OFF"), rcin->kill ? QColor(241, 210, 210) : QColor(238, 238, 238));
  setBadge(
    sub_mode_,
    QString("Sub: ") + (rcin->sub_mode ? "ON" : "OFF"),
    rcin->sub_mode ? QColor(220, 236, 255) : QColor(238, 238, 238));
  setMode(rcin->mode);
}
}  // namespace rcin
}  // namespace ctrl
}  // namespace gui
}  // namespace tobas
