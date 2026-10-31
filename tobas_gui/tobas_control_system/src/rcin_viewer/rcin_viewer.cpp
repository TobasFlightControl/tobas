// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_control_system/rcin_viewer/rcin_viewer.hpp"

#include <QFrame>
#include <QHBoxLayout>
#include <QPalette>
#include <QVBoxLayout>

#include <tobas_qt_tools/color.hpp>
#include <tobas_qt_tools/util.hpp>

namespace tobas::gui::ctrl::rcin
{
RCInputViewerWidget::RCInputViewerWidget(const rqt::RosQtBridge& _bridge)
{
  yaw_pitch_ = new StickPanel("Yaw / Pitch", "Yaw", "Pitch");
  roll_throttle_ = new StickPanel("Roll / Throttle", "Roll", "Throttle");

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

  const auto cols = new QHBoxLayout(this);
  cols->addWidget(yaw_pitch_, 1);
  cols->addWidget(roll_throttle_, 1);
  cols->addLayout(status_rows);

  // Connection
  connect(&_bridge, &rqt::RosQtBridge::rcInputReceived, this, &self::rcInputCb, Qt::QueuedConnection);
}

void RCInputViewerWidget::reset()
{
  yaw_pitch_->reset();
  roll_throttle_->reset();
  setBadge(enable_, "---", qt::color::gray90());
  setBadge(kill_, "Kill: ---", qt::color::gray90());
  setBadge(sub_mode_, "Sub: ---", qt::color::gray90());
  setMode(static_cast<FlightMode>(-1));
}

QLabel* RCInputViewerWidget::makeBadge(const QString& _text) const
{
  const auto badge = new QLabel(_text);
  badge->setAlignment(Qt::AlignCenter);
  badge->setAutoFillBackground(true);
  badge->setFrameStyle(QFrame::StyledPanel | QFrame::Sunken);
  badge->setMinimumHeight(30);
  badge->setMinimumWidth(70);
  return badge;
}

void RCInputViewerWidget::setBadge(QLabel* _badge, const QString& _text, const QColor& _background)
{
  _badge->setText(_text);

  auto pal = _badge->palette();
  pal.setColor(QPalette::Window, _background);
  pal.setColor(QPalette::WindowText, palette().color(QPalette::WindowText));
  _badge->setPalette(pal);
}

void RCInputViewerWidget::setMode(FlightMode _mode)
{
  setBadge(acrobat_, "Acrobat", _mode == FlightMode::kAcrobat ? qt::color::lightBlue() : qt::color::gray90());
  setBadge(stabilize_, "Stabilize", _mode == FlightMode::kStabilize ? qt::color::lightBlue() : qt::color::gray90());
  setBadge(loiter_, "Loiter", _mode == FlightMode::kLoiter ? qt::color::lightBlue() : qt::color::gray90());
}

void RCInputViewerWidget::rcInputCb(const tobas_msgs::RCInput::ConstSharedPtr& _rcin)
{
  if (_rcin->status != tobas_msgs::msg::RCInput::STATUS_OK) {
    reset();
    return;
  }

  yaw_pitch_->setValues(-_rcin->yaw, _rcin->pitch, true, _rcin->enable);
  roll_throttle_->setValues(_rcin->roll, _rcin->throttle, true, _rcin->enable);
  setBadge(
    enable_, _rcin->enable ? "Enabled" : "Disabled", _rcin->enable ? qt::color::lightBlue() : qt::color::gray90());
  setBadge(
    kill_, QString("Kill: ") + (_rcin->kill ? "ON" : "OFF"), _rcin->kill ? qt::color::lightRed() : qt::color::gray90());
  setBadge(
    sub_mode_,
    QString("Sub: ") + (_rcin->sub_mode ? "ON" : "OFF"),
    _rcin->sub_mode ? qt::color::lightBlue() : qt::color::gray90());
  setMode(_rcin->mode);
}
}  // namespace tobas::gui::ctrl::rcin
