// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_control_system/rcin_viewer/rcin_viewer.hpp"

#include <QFrame>
#include <QHBoxLayout>
#include <QPalette>
#include <QVBoxLayout>

#include <tobas_qt_tools/color.hpp>
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
  yaw_pitch_ = new StickPanel("Yaw / Pitch", "Yaw", "Pitch");
  roll_throttle_ = new StickPanel("Roll / Throttle", "Roll", "Throttle");

  kill_ = makeBadge("Kill: ---");
  sub_mode_ = makeBadge("Sub: ---");
  acrobat_ = makeBadge("Acro");
  acrobat_->setToolTip("Acrobat");
  stabilize_ = makeBadge("Stab");
  stabilize_->setToolTip("Stabilize");
  loiter_ = makeBadge("Loit");
  loiter_->setToolTip("Loiter");

  // Layout
  const auto switch_cols = new QHBoxLayout();
  switch_cols->addWidget(kill_, 1);
  switch_cols->addWidget(sub_mode_, 1);

  const auto mode_cols = new QHBoxLayout();
  mode_cols->setSpacing(0);
  mode_cols->addWidget(acrobat_, 1);
  mode_cols->addWidget(stabilize_, 1);
  mode_cols->addWidget(loiter_, 1);

  const auto status_rows = new QVBoxLayout();
  status_rows->setSpacing(0);
  status_rows->addLayout(switch_cols, 5);
  status_rows->addLayout(mode_cols, 8);

  const auto cols = new QHBoxLayout();
  cols->setContentsMargins(2, 0, 2, 0);
  cols->setSpacing(4);
  cols->addWidget(yaw_pitch_, 3);
  cols->addWidget(roll_throttle_, 3);
  cols->addLayout(status_rows, 1);
  setLayout(cols);

  // Connection
  connect(&bridge, &RosQtBridge::rcInputReceived, this, &self::rcInputCb, Qt::QueuedConnection);
}

void RCInputViewerWidget::reset()
{
  yaw_pitch_->reset();
  roll_throttle_->reset();
  setBadge(kill_, "Kill: ---", qt::color::gray90());
  setBadge(sub_mode_, "Sub: ---", qt::color::gray90());
  setMode(static_cast<FlightMode>(-1));
}

QLabel* RCInputViewerWidget::makeBadge(const QString& text) const
{
  const auto badge = new QLabel(text);
  badge->setAlignment(Qt::AlignCenter);
  badge->setAutoFillBackground(true);
  badge->setFrameStyle(QFrame::StyledPanel | QFrame::Sunken);
  badge->setMinimumWidth(44);
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
  setBadge(acrobat_, "Acro", mode == FlightMode::kAcrobat ? qt::color::lightBlue() : qt::color::gray90());
  setBadge(stabilize_, "Stab", mode == FlightMode::kStabilize ? qt::color::lightBlue() : qt::color::gray90());
  setBadge(loiter_, "Loit", mode == FlightMode::kLoiter ? qt::color::lightBlue() : qt::color::gray90());
}

void RCInputViewerWidget::rcInputCb(const tobas_msgs::RCInput::ConstSharedPtr& rcin)
{
  if (!rcin->ok) {
    reset();
    return;
  }

  yaw_pitch_->setValues(-rcin->yaw, rcin->pitch, true, rcin->enable);
  roll_throttle_->setValues(rcin->roll, rcin->throttle, true, rcin->enable);
  setBadge(
    kill_,
    QString("Kill: ") + (rcin->kill ? "ON" : "OFF"),
    rcin->enable && rcin->kill ? qt::color::lightRed() : qt::color::gray90());
  setBadge(
    sub_mode_,
    QString("Sub: ") + (rcin->sub_mode ? "ON" : "OFF"),
    rcin->enable && rcin->sub_mode ? qt::color::lightBlue() : qt::color::gray90());
  setMode(rcin->enable ? rcin->mode : static_cast<FlightMode>(-1));
}
}  // namespace rcin
}  // namespace ctrl
}  // namespace gui
}  // namespace tobas
